#!/usr/bin/python
# -*- coding: utf-8 -*-
# Copyright (C) 2016 Toyota Motor Corporation
from __future__ import print_function

import datetime
import distutils.util
import os
import signal
import socket
import subprocess
import sys
import threading
import time

import catkin.find_in_workspaces
import diagnostic_msgs.msg
import psutil
import rosgraph
import rosnode
import rospy
import std_msgs.msg
import tmc_msgs.msg
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

MOTOR_DEVICE = '/dev/ttyCTI2'
MESSAGES = {
    'HERO start': {'ja': (u"HEROスタート", 3.0),
                   'en': (u"HERO start", 2.0)},
    'Process died': {'ja': (u"停止しました", 3.0),
                     'en': (u"HERO stopped", 1.0)},
    'Killing all nodes': {'ja': (u"全てのノードを停止します", 4.0),
                          'en': (u"Killing all the nodes", 4.0)},
    'Failed to launch. Retrying..': {'ja': (u"起動に失敗しました。再起動します", 5.0),
                                     'en': (u'Failed to launch. Retrying', 4.0)
                                     },
    'Timeout. Start shutdown': {'ja': (u"{0}分経過しました", 3.0),
                                'en': (u"{0} minutes passed", 3.0)},
}


def wait_until(func, timeout=None, polling=1.0, *args, **kwargs):
    u"""Wait while polling while func satisfies the condition

    Timeout = None waiting indefinitely
    """
    if timeout is not None:
        end_time = time.time() + timeout
    else:
        end_time = time.time()
    while (time.time() < end_time or timeout is None):
        cond = func(*args, **kwargs)
        if cond:
            return True
        time.sleep(polling)
    return False


def exxx_read_hash(motor_id):
    u"""exxx_read_hash

    When communication succeeds it returns dict as follows
    {'firmware_hash': '0bd8ae8035a365c72dddf85e712117156c824372',
     'control_table_hash': '6fe89466421b08f50b5bb06e59f1fb09'}
    Returns None if it fails
    """
    try:
        exxx_read_hash_commnad = catkin.find_in_workspaces.find_in_workspaces(
            ['libexec'], 'hsrb_servomotor_protocol', 'exxx_read_hash')[0]
        output = subprocess.check_output([exxx_read_hash_commnad,
                                          MOTOR_DEVICE,
                                          str(motor_id)],
                                         stderr=subprocess.PIPE)
        if sys.version_info.major == 3:
            output = output.decode()
        output = {line.split('=')[0]: line.split('=')[1]
                  for line in output.replace(' ', '').split('\n')[:-1]}
        return output
    except subprocess.CalledProcessError:
        return None


def exxx_read_data_table(param, motor_id):
    u"""exxx_read_data_table

    Returns None if it fails
    """
    try:
        exxx_read_data_table_command = \
            catkin.find_in_workspaces.find_in_workspaces(
                ['libexec'], 'hsrb_servomotor_protocol',
                'exxx_read_data_table')[0]
        output = subprocess.check_output([exxx_read_data_table_command,
                                          MOTOR_DEVICE,
                                          str(motor_id),
                                          param],
                                         stderr=subprocess.PIPE)
        if sys.version_info.major == 3:
            output = output.decode()
        for line in output.split('\n'):
            if line.startswith(param):
                return line.split(':')[1].strip()
        return None
    except subprocess.CalledProcessError:
        return None


def get_lsusb_diag():
    u"""The result of lsusb DiagnosticStatus"""
    status = diagnostic_msgs.msg.DiagnosticStatus()
    status.name = "hsrb_robot_service: lsusb"
    status.hardware_id = 'USB'
    try:
        result = subprocess.check_output(["lsusb"])
        if sys.version_info.major == 3:
            result = result.decode()
        status.level = diagnostic_msgs.msg.DiagnosticStatus.OK
        status.message = "lsusb succeeded"
    except subprocess.CalledProcessError as err:
        status.level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
        status.message = "lsusb failed"
        result = err.output
    status.values.append(diagnostic_msgs.msg.KeyValue(
        key="result",
        value=result))
    return status


def get_motor_status_diag(param, motor_ids):
    u"""Read specified axis portion param DiagnosticStatus"""
    status = diagnostic_msgs.msg.DiagnosticStatus()
    status.name = "hsrb_robot_service: {0}".format(param)
    status.level = diagnostic_msgs.msg.DiagnosticStatus.OK
    status.hardware_id = 'motor {0}'.format(param)
    status.message = "exxx_read {0} succeeded".format(param)
    for motor_id in motor_ids:
        value = exxx_read_data_table(param, motor_id)
        if value is None:
            status.level = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            status.message = "exxx_read {0} failed".format(param)
            value = "Unknown"
        status.values.append(diagnostic_msgs.msg.KeyValue(
            key=str(motor_id),
            value=value))
    return status


def wait_until_servo_is_ready(servio_id, timeout=None):
    u"""Wait till you can communicate with the motor"""
    rospy.loginfo("Waiting until servo is ready")
    return wait_until(lambda: exxx_read_hash(servio_id) is not None,
                      timeout=timeout,
                      polling=0.5)


def wait_until_servo_is_not_ready(servio_id, timeout=None):
    u"""Wait until it can not communicate with the motor"""
    rospy.loginfo("Waiting until servo is not ready")
    return wait_until(lambda: exxx_read_hash(servio_id) is None,
                      timeout=timeout,
                      polling=0.5)


class RobotService(object):
    u"""Automatic startup control class"""

    SUBSCRIBE_TIMEOUT = 1.0
    # It is preferable to take it from hsrb_bringup/params/hsrb_hw_config.yaml
    MOTOR_IDS = [11, 12, 13, 21, 22, 23, 24, 25, 31, 32, 41]

    def __init__(self,
                 critical_nodes=['robot_hardware'],
                 all_nodes_pkg='hero_bringup',
                 all_nodes_launch_file='hero_start.launch',
                 hardware_pkg='hsrb_bringup',
                 hardware_launch_file='hsrb_bringup.launch',
                 remap_args=None,
                 watch_motor_id=11,
                 launch_monitor=True,
                 keep_nodes_time=10,
                 force_screen=False,
                 use_fast_restart=False,
                 lang='ja'):
        socket.setdefaulttimeout(5.0)
        self.name = 'robot_service'
        self.critical_nodes = critical_nodes
        self.all_nodes_pkg = all_nodes_pkg
        self.all_nodes_launch_file = all_nodes_launch_file
        self.hardware_pkg = hardware_pkg
        self.hardware_launch_file = hardware_launch_file
        if remap_args is None:
            remap_args = []
        self.remap_args = remap_args
        self.watch_motor_id = watch_motor_id
        self.launch_monitor = launch_monitor
        self.keep_nodes_time = keep_nodes_time
        self.force_screen = force_screen
        self.lang = lang
        self.master = rosgraph.Master(self.name)
        self.pids = {}
        self.proc_device = None
        self.proc_app = None
        self.talk_request_pub = None
        self.diag_pub = None
        self.button_off_count = 0
        self.previous_button_status = True
        self.lock = threading.RLock()
        self.use_fast_restart = use_fast_restart

    def __del__(self):
        rospy.loginfo("Destructor")
        self.term_nodes(True)

    def signal(self, sig, stack):
        u"""Signal handler"""
        if sig != self.previous_signal:
            # Ignore if the same signal comes multiple times while the node is stopped
            self.previous_signal = sig
            self.term_nodes(all_nodes=True)
        # ROS I do not use signal handlers of rospy signal_shutdown call
        rospy.signal_shutdown("signal-" + str(sig))
        sys.exit(sig)

    def wait_ready(self):
        u"""Boot preparation"""
        print("Waiting for the master")
        wait_until(self.master.is_online, polling=1.0)
        signal.signal(signal.SIGTERM, self.signal)
        signal.signal(signal.SIGINT, self.signal)
        rospy.loginfo("Connecting to /talk_request")
        self.talk_request_pub = rospy.Publisher(
            '/talk_request', tmc_msgs.msg.Voice, queue_size=1)
        result = wait_until(
            lambda: self.talk_request_pub.get_num_connections() > 0,
            timeout=self.SUBSCRIBE_TIMEOUT, polling=0.1)
        if not result:
            rospy.logerr("Failed to connect to /talk_request")
        diag_topic = rospy.names.resolve_name("diagnostics")
        rospy.loginfo("Connecting to {}".format(diag_topic))
        self.diag_pub = rospy.Publisher(
            'diagnostics', diagnostic_msgs.msg.DiagnosticArray,
            queue_size=10)
        result = wait_until(lambda: self.diag_pub.get_num_connections() > 0,
                            timeout=self.SUBSCRIBE_TIMEOUT, polling=0.1)
        if not result:
            rospy.logerr("Failed to connect to {}".format(diag_topic))
        rospy.Subscriber('runstop_button', std_msgs.msg.Bool,
                         self.runstop_button_cb)
        use_fast_restart = os.environ.get('USE_FAST_RESTART')
        if use_fast_restart:
            try:
                self.use_fast_restart = bool(distutils.util.strtobool(
                    use_fast_restart))
            except Exception as e:
                rospy.logwarn(e)
        if self.use_fast_restart:
            rospy.loginfo("USE_FAST_RESTART is enabled")

    def launch_nodes(self, all_nodes=False, timeout=15.0, polling=1.0):
        u"""Starting a node"""
        rospy.loginfo("Start launching")
        # ROS_LOG_DIR Only in the environment variable,
        # stdout/stderr Save to a file
        if 'ROS_LOG_DIR' in os.environ and not self.force_screen:
            run_id = rospy.get_param('/run_id')
            log_dir = os.path.join(os.environ['ROS_LOG_DIR'], run_id)
            date = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            stdout_file = open(os.path.join(
                log_dir, 'robot-{0}-stdout.log'.format(date)), 'w')
            stderr_file = open(os.path.join(
                log_dir, 'robot-{0}-stderr.log'.format(date)), 'w')
            stdout_file_dev = open(os.path.join(
                log_dir, 'robot-dev-{0}-stdout.log'.format(date)), 'w')
            stderr_file_dev = open(os.path.join(
                log_dir, 'robot-dev-{0}-stderr.log'.format(date)), 'w')
        else:
            stdout_file = None
            stderr_file = None
            stdout_file_dev = None
            stderr_file_dev = None
        self.pids = {}

        if self.launch_monitor:
            remap = {'monitor': 'true'}
        else:
            remap = {'monitor': 'false'}
        remap['auto_start'] = 'true'

        ## Get and set robot version
        #robot_version = os.environ.get("ROBOT_VERSION")
        #robot_name = robot_version.replace('"', '').split('-')[0].lower()
        #if robot_name not in ('hsrb', 'hsrc'):
            #raise Exception("Invalid ROBOT_VERSION:{0}".format(robot_version))

        # Start up nodes other than device type
        if all_nodes:
            self.proc_app = subprocess.Popen(
                ['/opt/ros/{0}/bin/roslaunch'.format(os.environ['ROS_DISTRO']),
                 self.all_nodes_pkg,
                 self.all_nodes_launch_file] +
                ["{0}:={1}".format(key, value) for key, value
                 in remap.items()] +
                self.remap_args,
                stdout=stdout_file,
                stderr=stderr_file,
                close_fds=True)
        # Launch device node
        self.proc_device = subprocess.Popen(
            ['/opt/ros/{0}/bin/roslaunch'.format(os.environ['ROS_DISTRO']),
             self.hardware_pkg,
             self.hardware_launch_file] +
            self.remap_args,
            stdout=stdout_file_dev,
            stderr=stderr_file_dev,
            close_fds=True)

        def nodes_ready():
            try:
                pids = {node: ServerProxy(rosnode.get_api_uri(
                    self.master, node, skip_cache=True)).getPid(self.name)
                    for node in self.critical_nodes}
                if all([pid[0] for pid in pids.values()]):
                    self.pids = {node: pid[2] for node, pid in pids.items()}
                    return True
            except Exception:
                pass
            return False
        return wait_until(nodes_ready, timeout=timeout, polling=polling)

    def watch(self):
        u"""Monitoring nodes"""
        rospy.loginfo("Monitoring following nodes:" +
                      "".join(["[%s:%d]" % (node, pid)
                               for node, pid in self.pids.items()]))

        def is_any_dead():
            for node, pid in self.pids.items():
                try:
                    os.kill(pid, 0)
                except OSError:
                    rospy.loginfo("%s[%d] is dead", node, pid)
                    return True
            return False
        wait_until(is_any_dead, timeout=None, polling=1.0)

    def notify(self, message, wait, *args, **kwargs):
        u"""Speech notification"""
        rospy.loginfo(message)
        if message in MESSAGES:
            if self.lang in ('ja', 'en'):
                sentence = MESSAGES[message][self.lang][0].format(
                    *args, **kwargs)
                self.talk_request_pub.publish(
                    tmc_msgs.msg.Voice(sentence=sentence,
                                       language={'ja': 0, 'en': 1}[self.lang]))
                if wait:
                    time.sleep(MESSAGES[message][self.lang][1])

    def record_states(self):
        u"""Take necessary logs before starting

        1. lsusb
        2. present_log_power_cycle
        3. present_log_motor_rev_total
        """
        diag = diagnostic_msgs.msg.DiagnosticArray()
        diag.header.stamp = rospy.Time.now()
        status = get_lsusb_diag()
        diag.status.append(status)
        status = get_motor_status_diag('present_log_power_cycle',
                                       self.MOTOR_IDS)
        diag.status.append(status)
        status = get_motor_status_diag('present_log_motor_rev_total',
                                       self.MOTOR_IDS)
        diag.status.append(status)
        self.diag_pub.publish(diag)

    def kill_process(self, proc):
        u"""Drop all processes including grandchild process"""
        try:
            parent = psutil.Process(proc.pid)
        except psutil.NoSuchProcess:
            return
        # Retrieve all child processes recursively
        children = parent.children(recursive=True)
        # Drop target process
        proc.terminate()
        proc.wait()
        # Terminate the remaining child process
        for child in children:
            if child.is_running():
                child.kill()

    def term_nodes(self, all_nodes=False):
        u"""Stop the node"""
        if all_nodes:
            if self.proc_app:
                self.notify('Killing all nodes', True)
                self.kill_process(self.proc_app)
                self.proc_app = None
        if self.proc_device:
            self.kill_process(self.proc_device)
            self.proc_device = None

    def runstop_button_cb(self, msg):
        u"""
        Subscriber's callback function subscribing to the state of emergency stop button
        Holds the number of times the state of the emergency stop button has changed from on to off.
        :param msg:
        """
        if not msg.data and self.previous_button_status:
            with self.lock:
                self.button_off_count += 1
        self.previous_button_status = msg.data

    def run(self):
        u"""Main loop"""
        self.wait_ready()
        wait_until_servo_is_not_ready(self.watch_motor_id)
        is_all = True
        while True:
            self.term_nodes(is_all)
            # Signal state reset
            self.previous_signal = None
            wait_until_servo_is_ready(self.watch_motor_id)
            with self.lock:
                self.button_off_count = 0
            self.notify('HERO start', True)
            self.record_states()
            success = self.launch_nodes(is_all)
            # When an unintended behavior occurs, set it to True in order to restart all nodes
            is_all = True
            if success:
                self.watch()
                self.notify('Process died', False)
                if exxx_read_hash(self.watch_motor_id) is not None:
                    rospy.loginfo('Relay is on')
                else:
                    rospy.loginfo('Relay is off')
                detected = wait_until(lambda: self.button_off_count > 0,
                                      timeout=60.0 * self.keep_nodes_time,
                                      polling=0.1)
                rospy.loginfo("Detected: {}".format(detected))
                # When there is no operation for a certain period of time, all the nodes are stopped
                if not detected:
                    self.notify('Timeout. Start shutdown', True,
                                self.keep_nodes_time)
                    continue
                # Button on / off within 2 seconds Restart all nodes when detecting more than 2 times
                if self.use_fast_restart:
                    is_all = wait_until(lambda: self.button_off_count >= 3,
                                        timeout=2.0, polling=0.1)
            else:
                self.notify('Failed to launch. Retrying..', True)
