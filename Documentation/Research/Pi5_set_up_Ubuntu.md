# Initial set up of the RPi5 with Ubuntu Server for the Grabby Project

---

**Project:** Grabby\
**Author:** Julius Ortstadt\
**Date:** 08.11.2025\
**Description:** This is a practical guide on how to set up the RaspberryPi 5 for the Grabby project. Used is Ubuntu 24.04.3 LTS Server to support ROS2 Jazzy and ROS1 Melodic in a Docker container and connect to it with no need of a screen. 

---

## Table of Contents
- [Initial set up of the RPi5 with Ubuntu Server for the Grabby Project](#initial-set-up-of-the-rpi5-with-ubuntu-server-for-the-grabby-project)
  - [Table of Contents](#table-of-contents)
  - [Login credentials](#login-credentials)
  - [Connect to the RPi with FileZilla](#connect-to-the-rpi-with-filezilla)
  - [Hardware Information](#hardware-information)
  - [Software Information](#software-information)
  - [Practical Information](#practical-information)
  - [OS Installation on the RPi5](#os-installation-on-the-rpi5)
  - [Set up NoMachine](#set-up-nomachine)
  - [Minicom](#minicom)
  - [Arduino IDE](#arduino-ide)
  - [WiFi config](#wifi-config)
  - [Troubleshooting](#troubleshooting)
    - [NoMachine](#nomachine)
    - [Configure the Ethernet port](#configure-the-ethernet-port)
    - [SSH problems after firewall reset](#ssh-problems-after-firewall-reset)
    - [SSH problem for Ethernet connection](#ssh-problem-for-ethernet-connection)
  - [References](#references)


## Login credentials
**Device name:** rpi\
**Username:** rpi\
**Password:** rpi 

## Connect to the RPi with FileZilla 
**Server:** sftp://<PI_IP>\
**Username:** rpi\
**Password:** rpi

## Hardware Information
- RaspberryPi 5 with 16Gb RAM
- Official RaspberryPi power supply
- 64Gb micro-SD card 
- micro-SD card adapter to SD if no micro-SD slot available on the PC

## Software Information
For this process, different software will be used. If relevant, the use and set up of each will be detailed in the relevant section of this documentation.

- **RaspberryPi Imager** for OS installation.
- **FileZilla** for file transfers between the RPi and another PC (based on SSH).
- **Advanced IP Scanner** to determine the IP address of a device in a network. Note that any other IP scan software can be used.
- **NoMachine** to have a remote desktop connection between a PC and the RPi and use it in headless mode

## Practical Information
For better readability, the following will be done:
- RaspberryPi (5) will be referred to as RPi (5)

## OS Installation on the RPi5
An official guide can be found on the [Ubuntu Tutorial page](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview).
Since we install Ubuntu Server, there is no need to diretly connect to a screen with HDMI and a mouse and keyboard etc. The Pi will directly be setup in headless mode.
- Use of **RaspberryPi Imager**
    - Choose the RPi model
    - Choose the OS. In this case, Ubuntu 24.04.3 LTS Server.
    - Select the SD Card
    - In the Advanced Settings (after clicking **Next**)
        - Set up the username and password
        - Set up the Wi-Fi credentials if available
        - Enable SSH support
- When done, insert the micro-SD card into the RPi and connect it to power to boot it.
- Wait a few seconds for the RPi to do the first boot, connect to the internet etc.
- Using **Advanced IP Scanner**, find the IP of the Pi in the current network. Note that the PC and the Pi need to be in the same network.
- Connect via SSH with Windows Powershell
    ```bash
    ssh username@hostname_or_IP
    ```
    Here this will be: 
    ```bash
    ssh rpi@rpi
    ```

Since Ubuntu Server is used, no desktop is available. However, one will be needed to visualize the mapping and navigation procedure.
To do that, a desktop can be installed.
- When the connection via SSH is established, ensure the packages are updated to the latest version.
    ```bash
    sudo apt update
    sudo apt upgrade
    ```
- Chosen was XUbuntu as the desktop environment but other more lightweight choices exist like LUbuntu:
    ```bash
    sudo apt install xubuntu-desktop
    ```
- When installation is finished, reboot the Pi:
    ```bash
    sudo reboot
    ```

## Set up NoMachine
To connect to the remote desktop that was just installed, NoMachine will be used. 
Here is the detailled procedure to get everything up and running:
- From the [official NoMachine website](https://www.nomachine.com/) on the PC, download the latest installer for the ARM64 64-bit architecture (which is the one the Pi has).
At the time of writting, this was: **NoMachine for ARM DEB (arm64) / Version 9.2.18_3 for Ubuntu, Debian**. It is important to choose the one for Ubuntu.
    - Go to **Home -> Download -> NoMachine for ARM**
- In the home folder, a download folder was created to where the installer file will be copied:
    ```bash
    mkdir Downloads
    ```
- Transfer the file with FileZilla to the Pi.
  - Server: sftp://<Pi_IP>
  - Username: <Pi_Username>
  - Password: <Pi_Password>  
- Install the package on the Pi through SSH
    ```bash
    sudo dpkg -i nomachine_9.x.x_x_arm64.deb
    ```
- Run this command to fix any broken installation
    ```bash
    sudo apt --fix-broken install -y
    ```
- When this is done, check and start the NoMachine server
    ```bash
    sudo systemctl status nxserver
    sudo systemctl enable nxserver
    sudo systemctl start nxserver
    ```
The first command should already show something like **active (running)**. If so, the other commands don't need to be executed.

- Connect from the PC:
  - Install NoMachine on the PC by downloading the latest installer for the PC's OS.
  - Open NoMachine client on the PC.
  - Follow the instruction pages.
  - Connect to the Pi using its IP.
  - Log in with the Pi's username and password.
  - If NoMachine throws an error that no display was found and wants to create a virtual one, say **yes**.
  - If all went well, the desktop should appear. If not, go to the **Troubleshooting** section of this guide.

## Minicom
Install Minicom on the RPi5 to monitor traffic on the Serial port.
```bash
sudo apt install minicom
```

## Arduino IDE
To install the Arduino IDE (Legacy Edition) on the RPi5, follow the steps below:
- Go to the [official arduino website](https://www.arduino.cc/en/software/#ide) and download the *.tar.xz* file for the **ARM64 architecture (aarch64)**.
- Transfer the file to the RPi using FileZilla
- Go to the location of the file on the RPi and extract it.
```bash
tar -xf arduino-1.8.19-linuxaarch64.tar.xz
```
- Go inside the newly created folder
```bash
cd arduino-1.8.19/
```
- Install
```bash
sudo ./install.sh
```
- When done, an icon will appear under **Applications** on the desktop environment. The Arduino IDE is now ready to be used.

## WiFi config
If it is necessary to add a new WiFi network so that the Pi automatically connects to it, the following can be done if **netplan** is used.
1. Connect via SSH with the Pi (done here using an ethernet cable)
2. Edit the netplan file:
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
3. Update it like this (example adds a second Wi-Fi):
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      access-points:
        "Network1_Name":
          hidden: true
          auth:
            key-management: "psk"
            password: "password1"
        "Network2_Name":
          auth:
            key-management: "psk"
            password: "password2"
```
* `hidden: true` is only needed if the SSID is hidden
* Netplan will automatically connect to **any available known network**

4. Apply the changes
```bash
sudo netplan generate
sudo netplan apply
```
5. Verify
```bash
ip a show wlan0
iw dev wlan0 link
```

**Note**
* It is possible to set WiFi priority:
Netplan uses signal strength by default, but it is possible to **force priority**:
(Higher number = preferred network)

```yaml
access-points:
  "Powerphone":
    priority: 100
    hidden: true
    auth:
      key-management: "psk"
      password: "1"
  "MyHomeWiFi":
    priority: 50
    auth:
      key-management: "psk"
      password: "MySecurePassword"
```


## Troubleshooting
### NoMachine
If the connection to the Pi with NoMachine showed an error prompting to logout because a fatal error occured, the issue will be resolved in this section.
The config of NoMachine needs to be modified because by default it tries to start **GNOME** but the installed environment uses **Xfce**. This needs to be changed in the config and it should all work.
- Edit the NoMachine config
    ```bash
    sudo nano /usr/NX/etc/node.cfg
    ```
- Find the line
    ```bash
    DefaultDesktopCommand "/etc/X11/Xsession gnome-session"
    ```
- Change it to
    ```bash
    DefaultDesktopCommand "/usr/bin/startxfce4"
    ```
- Save and exit the file
- Restart NoMachine
    ```bash
    sudo systemctl restart nxserver
    ```
- Connect again with NoMachine
    - If it asks to create a new display -> **Yes**.

The Xfce desktop should now appear.

### Configure the Ethernet port
If an Ethernet cable is plugged in, by default it should get an IP address and auto-configure the port. However, if this is not the case, the steps below solve the problem that occured in the scope of this project.
Note that the cable connection is between the Pi and a router, not a PC.
In the projects case, the Ethernet port was not configured and therefore no IP address was assigned.

- SSH into the Pi. This supposes that a wireless connection (over WiFi can be established with the Pi)
- Check the IP addresses
    ```bash
    ip a
    ```
- Here the result was:
    ```bash
    2: eth0: <BROADCAST,MULTICAST> mtu 1500 qdisc noop state DOWN group default qlen 1000 link/ether 88:a2:9e:26:76:85 brd ff:ff:ff:ff:ff:ff
    ```
    Showing that the Ethernet was DOWN.
- Check whether Ethernet is managed by Netplan
    ```bash
    ls /etc/netplan
    ```
    It should return something like **50-cloud-init.yaml**
- Open this file
    ```bash
    sudo nano /etc/netplan/50-cloud-init.yaml
    ```
- Ensure that **eth0** is defined. If not, add this:
    ```yaml
    network:
      version: 2
      renderer: networkd

      ethernets:
        eth0:
          dhcp4: true
          optional: true
    ```
    **Note:** Since this is a *.yaml* file, formatting is very important. Tabs can't be used, everything needs to be spaces. Each indentation is actually 2 spaces.
- Save and exit the file.
- Apply the changes
    ```bash
    sudo netplan apply
    ```
    Or if there should be a debug output:
    ```bash
    sudo netplan --debug apply
    ```
- Check if Ethernet came UP
    ```bash
    ip a
    ```
    There should now be a line like this:
    ```bash
    2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> ...
        inet 192.168.x.x/24
    ```
    This now means that everything is up and running.

### SSH problems after firewall reset
If the firewall was deactivated and reactivated it may happen that the SSH port is blocked. 
This entry gives a step-by-step guide on resolving this issue.

**Note:** As no SSH connection was available, this procedure was done through the NoMachine remote desktop connection as it is another protocol (NX) and was still working. 

- Check the status of the SSH port
    ```bash
    sudo systemctl status ssh
    ```
    If it says **inactive** or **failed**, the connection needs to be re-established.
- To re-activate the SSH:
    ```bash
    sudo systemctl enable ssh
    ```
- Check the status. It should now say **active** or something similar.
    ```bash
    sudo systemctl start ssh
    ```
The SSH connection should now work again.


### SSH problem for Ethernet connection
If the Pi and the PC are connected directly with an Ethernet cable, establishing the SSH connection may not work directly.
Follow the steps below to solve the problem:
- On the windows PC enable network sharing
  - Go to **Network connections** in the control panel.
  - If connected to the Internet through WiFi, right-click the Wi-Fi adapter and go to **Properties**
  - Go to **Sharing tab**
  - Enable **Allow other network users to connet through this computer's internet connection**
  - Choose the Ethernet adapter as the shared adapter. This will automatically assign the PC's ethernet and the Pi an IP in the same network.
- SSH into the Pi. The IP of the Pi can be found using **Advanced IP Scanner**. It should be something like **192.168.137.x**
    ```bash
    ssh <username>@<Pi_IP>
    ```
- If an error occurs like **Warning: Remote Host Identification has changed** and the connection is not established, this is due to the fact that the PC still has an old SSH fingerprint of the Pi stored (created by connecting wirelessly etc.). But now that the connection method has changed and that it has a new IP, the fingerprint doesn't match. To solve this issue, the old fingerprint needs to updated/removed:
  - In Windows Powershell
    ```bash
    ssh-keygen -R <Pi_IP>
    ```
  - SSH into the Pi
    ```bash
    ssh <username>@<Pi_IP>
    ```
  - If it asks **Are you sure you want to continue connecting (yes/no/[fingerprint])?**, say yes.
  - The SSH connection should now be established.


**Note:** If everything was configured correctly, the Ethernet cable connection should not only allow for SSH connection but also provides internet access to the RPi5 and allows a connection via NoMachine to visualize the running desktop environment.


## References
- Ubuntu Server Tutorial
https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview

- NoMachine Installation guide
https://kb.nomachine.com/DT04U00269#2.5

- NoMachine Download page for ARM
https://download.nomachine.com/download/?id=30&platform=linux&distro=arm

- NoMachine Download page for Windows:
https://download.nomachine.com/download/?id=3&platform=windows
