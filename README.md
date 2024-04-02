# hero_bringup

[![CI](https://github.com/tue-robotics/hero_bringup/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/hero_bringup/actions/workflows/main.yml)

Launch, machine and parameter files required to bringup the HERO robot

## HERO1

### (Re-)install HERO

#### Before clean install

- Upload calibration files: `hsrb_command upload_robot_config` (as root/administrator)
- Back-Up of virtualbox image which are located in `~/vbox_images`:
  - Stop systemd service: `sudo systemctl stop hero1-speech5-windows-speech.service` or `sudo systemctl stop hero1-demo-speech5-windows-speech.service`
  - Actual back-up i.e.: `scp -r ~/vbox_images amigo@hero2.local:~/vbox_images_backup`
- Back-Up the odometer files: `scp -r ~/odometer amigo@hero2.local:~/odometer_backup`

#### Clean install

- Required items:
  - USB Stick
  - Optional: External monitor with VGA connection. This is needed for any changes in the BIOS. As the BIOS is not shown on the internal screen.
- Insert keyboard into robot
- Insert USB Stick directly into the robot. Don't use any USB hub, etc.
- Boot the robot. If no install screen is shown, follow these steps:

  - Reboot the robot, press `ESC` during boot to enter the BIOS.
  - Check in `Boot` if the USB stick is set as first boot option. **Note:** No UEFI.
  - Otherwise change in `CMS parameters` or `Hard Drive parameters` the `Boot option filter` to `legacy only`.
  - Save changes and reboot again into the bios. Check again the USB stick is the first boot option. This should be the case. Reboot and let the installer do its job.

- Wait till installer is finished. It will display a message to remove install medium. Do this and press main power button for a few seconds to shutdown the robot.
- Start the robot again via the main power button.

#### After clean install

Directly on the robot:

- Open a terminal and change to `administrator`: `su - administrator`
- Change hostname to `hero1`:
  - `sudo vim /etc/hostname`
  - `sudo vim /etc/hosts`
- Set static IP (Using external Wi-Fi adapter):
  - Make a back-up of the config: `sudo cp /etc/network/interfaces.eth /etc/network/interfaces.eth.bk`
  - Open the file: `sudo vim /etc/network/interfaces.eth`
  - Look for the following line: `iface enp4s0 inet dhcp`. Change it to: `iface enp4s0 inet static`
  - Add the following lines directly after the previous changed line:

    ```default
    address 192.168.44.51
    netmask 255.255.255.0
    gateway 192.168.44.1
    dns-nameservers 192.168.44.1 8.8.8.8 8.8.4.4
    ```

  - Make a back-up of the resolve config: `sudo cp /etc/systemd/resolved.conf /etc/systemd/resolved.conf.bk`
  - Open the file: `sudo vim /etc/systemd/resolved.conf`
  - Change the line: `#DNS=` to `DNS=192.168.44.1 8.8.8.8 8.8.4.4`
  - Set to use ethernet: `sudo disable_wireless`
  - reboot: `sudo reboot`

The following steps need to be done on the robot(`hsr-hmi`) via the GUI:

- Disable automatic updates in settings: *Software & Updates* > *Updates*:
  - *Automatically check for updates*: `Never`
  - *When there are other updates*: `Display every two weeks`
  - *Notify me of a new Ubuntu version*: `Never`

The following steps can be done via SSH or directly on the robot (SSH: `ssh administrator@hero1.local`)

- Change passwords of the accounts (hsr-hmi, hsr-user, administrator): `sudo passwd XX`
- (Run apt-get update): `sudo apt-get update` (as root/administrator)
- Update hsrb_command: `hsrb_command upgrade` (as root/administrator)
- Update dockers: `hsrb_command update_release XX.XX` (as root/administrator)
- Restore calibration files: `hsrb_command restore` (as root/administrator)
- Restore odometer files: `scp -r amigo@hero2.local:~/odometer_backup ~/odometer`
- Fix the owner of all files and folders in the `administrator` home folder: `sudo chown -R administrator:administrator /home/administrator/*`
- restore virtualbox image to `~/vbox_images`: `scp -r amigo@hero2.local:~/vbox_images_backup ~/vbox_images`(Path of the files should be like: `~/vbox_images/windows/windows.XX`)
- Install virtualbox debian from <https://www.virtualbox.org/wiki/Downloads>:
  - `sudo dpkg -i "XX.deb"`
  - (`sudo apt-get install -f` to fix missing dependencies)
- On the robot(`administrator`): Open virtualbox and add the windows image.
- On the robot(`administrator`): Set correct network adapter for the virtualbox. Depending on the use of internal of external Wi-Fi.
- On the robot(`hsr-hmi`): In `startup applications` disable Toyota head display
- Remove apt sources provided by Toyota: `sudo rm /etc/apt/sources.list.d/*`
- Install tue-env (<https://github.com/tue-robotics/tue-env>) and install `networking` target: `tue-get install networking`
- Reboot the robot to activate ssh: `sudo reboot`
- Use ssh with key forwarding to connect to the robot. This is needed to clone private repos. (`sshhero1` does this)
- Configure tue-env to use ssh for git: `tue-env config ros-$TUE_ROS_DISTRO git-use-ssh`
- Install the `hero1` target: `tue-get install hero1`
- Build the software: `tue-make`
- (Fix timezone: `tue-robocup-set-timezone-home`)
- Disable the default Toyota Services: `sudo systemctl disable docker.hsrb.*`
- Stop the default Toyota Services: `sudo systemctl stop docker.hsrb.*`
- Install the background services: `rosrun hero_bringup install_systemd_autostart_hero1`
- Reboot and ready to go!

### HERO display

#### First install

1. Download the latest `hero-display.AppImage` from <https://github.com/tue-robotics/hero-display/releases>
2. Move the file to `sudo mv ./hero-display.AppImage /opt/tue/bin/hero-display.AppImage` It might be needed to create the target directory: `sudo mkdir -p /opt/tue/bin`
3. Make sure all users have all rights: `sudo chmod a+rwx /opt/tue/bin/hero-display.AppImage`
4. On HERO (hsr-hmi user) go to **Startup Applications** and add a new item with `/opt/tue/bin/hero-display.AppImage` as command.
5. Run it once manually or reboot HERO

#### Update

1. Close the current hero-display session
2. Repeat steps 1-3 & 5 from [First install](#first-install)

### Services

The following services are running on HERO1. Normally these are ran from the demo account:

- **hero1-battery-conversion**: Publishing a BatteryState message from the custom Toyota messages
- **hero1-monitor.service**: HERO version of `hsrb_monitor` from `hsrb_bringup`
- **hero1-ntpdate.service**: Service to sync time with servers
- **hero1-webserver.service**: Service to always have the webserver running to facilitate hero-dashboard, hero-display and other web applications.
- **hero1-roscore.service**: Service containing the roscore. Runs the Toyota wrapper(`rosrun hsrb_bringup hsrb_roscore_service namespace:=/hero`)
- **hero1-speech1-vboxautostart-service**: Slightly changed version of vboxautostart
- **hero1-speech2-vboxballoonctrl-service**: Slightly changed version of vboxballoonctrl
- **hero1-speech3-vboxdrv**: Slightly changed version of vboxdrv
- **hero1-speech4-vboxweb-service**: Slightly changed version of vboxweb
- **hero1-speech5-windows-speech**: Service to start the Windows virtualbox

## HERO2

### (Re)-install

#### Before install

- Backup the backup of the virtualbox images: `scp -r amigo@hero2.local:~/vbox_images_backup ~/vbox_images_backup`
- Backup any other data (I.E. MEGA folder).

#### Install

- Install Ubuntu 20.04 Desktop:

  ```default
  Name: amigo
  Hostname: hero2
  Username: amigo
  ```

- After install go to TTY6(Ctrl+Alt+F6), install nvidia-driver-440(Or the one matching the cuda version used for openpose) and reboot: `sudo apt-get install nvidia-driver-440` and `sudo reboot`
- Change ethernet configuration to static IP via GUI. Set static IP for ipv4 with the following settings:

  ```default
  Address: 192.168.44.52
  Netmask: 255.255.255.0
  Gateway: 192.168.44.1
  DNS servers: 192.168.44.1, 8.8.8.8, 8.8.4.4
  ```

- Turn Airplane-mode on, make sure Wi-Fi and Bluetooth is disabled
- Set `Automatic suspend` to off in Power Settings, both for `Battery Power` and `Plugged in`
- Keep laptop on with lid closed: `sudo vim /etc/systemd/logind.conf`, add `HandleLidSwitch=ignore` and `HandleLidSwitchExternalPower=ignore`
- Install tue-env (<https://github.com/tue-robotics/tue-env>) and install `networking` target: `tue-get install networking`
- Configure tue-env to use ssh for git: `tue-env config ros-$TUE_ROS_DISTRO git-use-ssh`
- Reboot the laptop to activate ssh: `sudo reboot`
- Use ssh with key forwarding to connect to the robot. This is needed to clone private repos. (`sshhero2` does this)
- Install `hero2` target `tue-get install hero2`
- Set keyboard shortcut `Ctrl+Alt+T` to terminator
- Restore any backed-up data
- Setup MEGA sync via GUI
- Build the software: `tue-make`
- Install the background services: `rosrun hero_bringup install_systemd_autostart_hero2`
- Reboot and ready to go!

### Services

The following services are running on HERO2. Normally these are ran from the demo account:

- **hero2-battery-manager**: Running a battery manager to let the robot speak in case of a low battery level
- **hero2-openpose**: Running OpenPose
- **hero2-rgbd-shm-server**: Running a `rgbd` shared memory server, so there is only one RGBD connection from HERO1 to HERO2
- **hero2-upower-ros**: Running `upower_ros` to publish the battery state of HERO2

### Power settings

To prevent HERO2 to shut down on unplugging the power supply, some power settings need to be changed. (<https://askubuntu.com/a/1132068>)

```bash
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type nothing
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type nothing
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout 0
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-timeout 0
gsettings set org.gnome.settings-daemon.plugins.power lid-close-ac-action nothing
gsettings set org.gnome.settings-daemon.plugins.power lid-close-battery-action nothing
# Copy settings from amigo account to gdm account
OLD_IFS=$IFS; IFS=$'\n'; for x in $(sudo -Hu amigo gsettings list-recursively org.gnome.settings-daemon.plugins.power); do eval "sudo -Hu gdm dbus-launch gsettings set $x"; done; IFS=$OLD_IFS; unset OLD_IFS
```

## HERO demo laptop

### (Re-)install

- Install Ubuntu 18.04 Desktop:

  ```default
  Name: demo
  Hostname: demo-laptop
  Username: demo
  ```

- Setup Wi-Fi to AtHome(-2.4)
- Install tue-env (<https://github.com/tue-robotics/tue-env>)
- Install `hero-demo-laptop` target: `tue-get install hero-demo-laptop`
- Compile the software: `tue-make`
- Set keyboard shortcut `Ctrl+Alt+T` to terminator
- Pin shortcuts of `hero-dashboard` and `hero-rviz`
- Disable laptop going to sleep and disable power button turning off the laptop
- Copy ssh keys to HERO: `hero-copy-my-id`

## Toyota software compatibility

HERO uses many of the Toyota launch structure. This requires compatibility of launch file arguments, frame names, namespacing, etc.. In case of incompatibility it can look like a local issue. To exclude any uncertainty a list of compatible package versions is maintained. There is a seperate list for the [robot](./toyota_robot_versions.yaml) and [dev pc's](./toyota_dev_versions.yaml).

`hero-start` contains a required node, which checks the currently installed set of Toyota software packages. In case of a package, which is not tracked, a warning is shown. In case of a tracked package with an incompatible version, an error is shown and `hero-start` will terminate.

In case of an incompatible package version, you can add the new package version(s) to the list manually or run the update script with the add option: `rosrun hero_bringup update_toyota_software_version --add`. In case this seems to run without any issues (node names, topic names, service names, frame names, etc.), you can open a PR to update the tracked compatibility list.
In case of any issues, the issues should be solved and the compatibility list should be reset. You can reset the compatible package version of the packages causing the issues manually or reset the entire list by running the update script with the 'reset' option: `rosrun hero_bringup update_toyota_software_version --reset`. You can try to fix (small) issues yourself, but it is recommended to ask [@MatthijsBurgh](https://github.com/MatthijsBurgh) to fix these issues.
