#!/usr/bin/env python
import rospy

import tf

def broadcast():
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "odom",
                     "map")

if __name__ == '__main__':
    rospy.init_node('stationary_broadcaster')
    while not rospy.is_shutdown():
        broadcast()
        rospy.sleep(1)
