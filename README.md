# hero_bringup
Launch, machine and parameter files required to bringup the HERO robot

## (Re-)install HERO
### Before clean install
- Upload calibration files: `hsrb_command upload_robot_config` (as root/administrator)
- Back-Up of virtualbox image which are located in `~/vbox_images`:
    - Stop systemd service: `sudo systemctl stop hero-windows-speech.service`
    - Actual back-up i.e.: `scp -r ~/vbox_images/* amigo@hero2.local:~/vbox_images_backup`

### Clean install
- Required items:
    - USB DVD drive
    - Install disk (17.10 works)
    - keyboard
- Insert keyboard into robot
- Connect DVD drive to a laptop to be able to open the drive.
- Insert disk in DVD drive.
- Make sure robot is powered-off.
- Connect DVD drive to robot. Place the DVD drive on the head.
- Power-on the robot.
- Wait till disk is ejected automatically. (Screen can turn off and on during installation)
- Disconnect DVD drive.
- Press enter to reboot the robot.

### After clean install
- Fix the wireless network directly on the robot:
    - `wpa_gui`(as root/administrator) to add wifi network. Make sure to select the `wlp3so` interface

The following steps can be done via SSH or directly on the robot (SSH: `ssh administrator@hsrb.local`)
- (Run apt-get update): `sudo apt-get update` (as root/administrator)
- Update hsrb_command: `hsrb_command upgrade` (as root/administrator)
- Update dockers: `hsrb_command update_release XX.XX` (as root/administrator)
- Restore calibration files: `hsrb_command restore` (as root/administrator)
- Change passwords of the accounts (hsr-hmi, hsr-user, administrator): `sudo password XX`
- Fix the owner of the following folders: `/home/administrator/.cache` and `/home/administrator/.config` by:
`sudo chown -R administrator:administrator /home/administrator/.cache` and `sudo chown -R administrator:administrator /home/administrator/.config`
- Change hostname to `hero1`:
    - `sudo vim /etc/hostname`
    - `sudo vim /etc/hosts`
    - reboot: `sudo reboot` (Use `ssh administrator@hero1.local` after this step)
- Set static IP (when using internal Wi-Fi):
    - Make a back-up of the config: `sudo cp /etc/network/interfaces.wlan /etc/network/interfaces.wlan.bk`
    - Open the file: `sudo vim /etc/network/interfaces.wlan`
    - Look for the following line: `iface wlp3so inet dhcp`. Change it to: `iface wlp3so inet static`
    - Add the following lines directly after the previous changed line:
        ```
        address 192.168.44.51
        netmask 255.255.255.0
        gateway 192.168.44.1
        dns-nameservers 192.168.44.1 8.8.8.8 8.8.4.4
        ```
    - reboot: `sudo reboot`
- Set ethernet (when using external router for Wi-Fi):
    - Set symbolic link to use ethernet: `sudo ln -sf /etc/network/interfaces.eth /etc/network/interfaces`
- restore virtualbox image to `~/vbox_images`. (Path of the files should be like: `~/vbox_images/windows/windows.XX`)
- Install virtualbox debian from https://www.virtualbox.org/wiki/Downloads:
    -  `sudo dpkg -i "XX.deb"`
    - (`sudo apt-get install -f` to fix missing dependencies)
- Open virtualbox ON the robot and add the windows image.
- Set correct network adapter for the virtualbox. Depending on the use of internal of external Wi-Fi.
- Install tue-env (https://github.com/tue-robotics/tue-env) and install hero1 target: `tue-get install hero1`
- Build the software: `tue-make`
- Go to `hero_bringup` package. Execute the following to stop the Toyota and VirtualBox services and start ours: `./scripts/install_systemd_autostart_hero1`
- (Fix timezone: `tue-robocup-set-timezone-home`)
- Reboot and ready to go!

## HERO display
### First install
1. Download the latest `hero-display.AppImage` from https://github.com/tue-robotics/hero-display/releases
2. Move the file to `/opt/tue/bin/hero-display.AppImage`
3. Make sure all users have all rights: `(sudo) chmod a+rwx /opt/tue/bin/hero-display.AppImage`
4. On HERO (hsr-hmi user) go to **Startup Applications** and add a new item with `/opt/tue/bin/hero-display.AppImage` as command.
5. Run it once manually or reboot HERO

### Update
1. Close the current hero-display session
2. Repeat steps 1-3 & 5 from [First install](#first-install)


## HERO1
### Services
The following services are running on HERO1. Normally these are runned from the demo account:
- **hero1-battery-conversion**: Publishing a BatteryState message from the custom Toyota messages
- **hero1-monitor.service**: HERO version of `hsrb_monitor` from `hsrb_bringup`
- **hero1-ntpdate.service**: Service to sync time with servers
- **hero1-phone-ui.service**: Service to always have the webserver running to facilitate hero-dashboard, hero-display and other web applications.
- **hero1-roscore.service**: Service containing the roscore. Runs the Toyota wrapper(`rosrun hsrb_bringup hsrb_roscore_service namespace:=/hero`)
- **hero1-speech1-vboxautostart-service**: Slightly changed version of vboxautostart
- **hero1-speech2-vboxballoonctrl-service**: Slightly changed version of vboxballoonctrl
- **hero1-speech3-vboxdrv**: Slightly changed version of vboxdrv
- **hero1-speech4-vboxweb-service**: Slightly changed version of vboxweb
- **hero1-speech5-windows-speech**: Service to start the Windows virtualbox

## HERO2
### Services
The following services are running on HERO1. Normally these are ran from the demo account:
- **hero2-battery-manager**: Running a battery manager to let the robot speak in case of a low battery level
- **hero2-openpose**: Running OpenPose
- **hero2-rgbd-shm-server**: Running a `rgbd` shared memory server, so there is only one RGBD connection from HERO1 to HERO2
- **hero2-upower-ros**: Running `upower_ros` to publish the battery state of HERO2

### Power settings
To prevent HERO2 to shutdown on unplugging the power supply, some power settings need to be changed. (https://unix.stackexchange.com/questions/85251/my-laptop-shuts-down-every-time-i-unplug-it)

```bash
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type nothing
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type nothing
```
