# Setup of the RPi5 with ROS1 Melodic for the Grabby Project

---

**Project:** Grabby\
**Author:** Julius Ortstadt\
**Date:** 03.12.2025\
**Description:** This is a continuation of the practical guide on how to set up the RaspberryPi 5 for the Grabby project. This tutorial covers the topics on installing and setting up ROS1 Melodic for the project.

---

## Table of contents
- [Setup of the RPi5 with ROS1 Melodic for the Grabby Project](#setup-of-the-rpi5-with-ros1-melodic-for-the-grabby-project)
  - [Table of contents](#table-of-contents)
  - [Prerequisites](#prerequisites)
  - [Login credentials](#login-credentials)
  - [Install Docker](#install-docker)
  - [Pull the Docker image](#pull-the-docker-image)
  - [Run the Docker container](#run-the-docker-container)
    - [Start command](#start-command)
    - [Control a Docker container](#control-a-docker-container)
    - [Docker ROS Melodic Container Command Breakdown](#docker-ros-melodic-container-command-breakdown)
  - [Arduino ROS library files](#arduino-ros-library-files)
    - [Additional configuration](#additional-configuration)
      - [.bashrc](#bashrc)
      - [Make bash shell sudo](#make-bash-shell-sudo)
      - [Install additional packages](#install-additional-packages)
  - [Troubleshooting](#troubleshooting)
    - [CMake version out-of-date](#cmake-version-out-of-date)
  - [References](#references)

## Prerequisites
The **Pi5_set_up_Ubuntu** guide needs to be completed before the tasks presented in this guide. 

## Login credentials
**Device name:** rpi\
**Username:** rpi\
**Password:** rpi 

## Install Docker
To install Docker, execute the following steps which can also be found [here](https://docs.docker.com/engine/install/ubuntu/):
- Set up Docker's **apt** repository:
```bash
# Add Docker's official GPG key:
sudo apt update
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
```

- Install the Docker packages
```bash
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

- Verify that Docker is running or start it
```bash
sudo systemctl status docker

sudo systemctl start docker
```

- Verify the installation by running the **hello-world** image:
```bash
sudo docker run hello-world
```
This command downloads a test image and runs it in a container. When the container runs, it prints a confirmation message and exits.


## Pull the Docker image
There is no official Docker image for ROS1 Melodic on an ARM64 architecture. 
Therefore, an custom image found [here](https://hub.docker.com/r/tiryoh/ros-melodic-desktop/tags) will be used.

- Pull the image on the RPi
```bash
docker pull --platform=linux/arm64 tiryoh/ros-melodic-desktop:latest
```
- Verify it pulled the correct image
```bash
docker image inspect tiryoh/ros-melodic-desktop:latest --format '{{.Os}}/{{.Architecture}}'
```
It should return **linux/arm64**


## Run the Docker container
### Start command
Run the image with the following command to make it have all the necessary rights, persistency, shared folder etc.
```bash
docker run -it --privileged \
  --network host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/ros_shared:/ros_shared" \
  --volume="/dev:/dev" \
  --name ros_melodic_container \
  tiryoh/ros-melodic-desktop:latest \
  bash
```

**Note:** The initial setup as well as the later installation of packages for ROS in the docker could have been done using a *docker compose* file. This would be an adjustment to consider for later development. 

### Control a Docker container
These commands are to be executed in a terminal on the host.
- To list all the Docker containers (active and stopped):
```bash
docker ps -a
```
- To list only the active Docker containers:
```bash
docker ps 
```
- To start a persistent container:
```bash
docker start -ai <container_name>
```

**Note:**\
-a -> attach to stdout/stderr\
-i -> interactive shell

- To start a container in the background
```bash
docker start <container_name>
```
- Exit the container. It stops (if it was the last terminal instance in the container) but stays saved.
```bash
exit
```
- To stop a container (gracefully stops the container but is the same as exiting)
```bash
docker stop <container_name>
```
- Link a new terminal to an already running container
```bash
docker exec -it <container_name> bash
```
- Reconnect the current terminal to an already running container
```bash
docker attach <container_name>
```
- Delete a container
```bash
docker rm <container_name>
```

### Docker ROS Melodic Container Command Breakdown

| Command / Option | Description |
|------------------|-------------|
| `docker run` | Creates and starts a new container from a Docker image. |
| `-it` | Runs the container in interactive mode with a TTY (`-i` keeps STDIN open, `-t` allocates a terminal). |
| `--privileged` | Grants extended privileges to the container, allowing full access to host devices (commonly required for robotics hardware and drivers). |
| `--network host` | Uses the host’s network stack directly, which simplifies ROS node communication and avoids port forwarding. |
| `--env="DISPLAY=$DISPLAY"` | Passes the host’s X11 display variable so GUI applications inside the container can render on the host. |
| `--env="QT_X11_NO_MITSHM=1"` | Disables MIT shared memory for Qt apps to prevent GUI issues when running Qt applications (e.g., RViz) in Docker. |
| `--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"` | Mounts the host’s X11 Unix socket into the container to enable GUI forwarding. |
| `--volume="$HOME/ros_shared:/ros_shared"` | Shares a directory between host and container for persistent storage and file exchange. |
| `--volume="/dev:/dev"` | Mounts the host’s `/dev` directory, allowing access to hardware devices such as USB, serial ports, and cameras. |
| `--name ros_melodic_container` | Assigns a readable name to the container for easier management. |
| `tiryoh/ros-melodic-desktop:latest` | The Docker image used: ROS Melodic Desktop with GUI tools, maintained by `tiryoh`. |
| `bash` | Starts an interactive Bash shell inside the container instead of the default command. |


## Arduino ROS library files
- Enter the ROS1 docker container
```bash
docker exec -it <container_name> bash
```
- Install **rosserial_arduino** inside the container
```bash
sudo apt update
sudo apt install ros-noetic-rosserial-arduino ros-noetic-rosserial
```
- Create the **ros_lib** folder
```bash
cd /tmp
rosrun rosserial_arduino make_libraries.py ros_lib
```
There should now be **/tmp/ros_lib/** with files like *ros.h* and folders like *std_msgs/Int16.h* etc.
- Copy **ros_lib** to the host 
```bash
cp -r /tmp/ros_lib /shared_folder/  
```
- Install ros_lib on the Raspberry Pi for Arduino IDE. On the host go to the shared folder and from there move the **ros_lib** folder.
```bash
mv ~/shared_folder/ros_lib ~/Arduino/libraries/ros_lib
```
- The final path must be:
```bash
~/Arduino/libraries/ros_lib/
    ros.h
    std_msgs/
    geometry_msgs/
    ...
```
- Restart the Arduino IDE and compile the code. It should work now.

**Note:** The **ros_lib** folder can be found in this github repository. There is therefore no need to do the previously mentioned steps if necessary.

### Additional configuration
#### .bashrc
- Add to *.bashrc* to auto-source the ROS1 Melodic environment. 
```bash
nano ~/.bashrc
source /opt/ros/melodic/setup.bash
```
- Forwards the display rights for Docker
```bash
xhost +local:root 
```

#### Make bash shell sudo
To make the shell sudo, run:
```bash
sudo su
```

#### Install additional packages
- Install **nano** in the docker
```bash
apt-get install nano
```
- Install **git** in the docker
```bash
apt-get install git
```

## Troubleshooting
### CMake version out-of-date
If the CMake version inside the docker is out-of-date, it can be updated.
1. Update
```bash
sudo apt update
sudo apt install -y build-essential libssl-dev wget tar
```
2. Pick a stable version >= 3.16 (here 3.26.4)
```bash
cd /tmp
wget https://github.com/Kitware/CMake/releases/download/v3.26.4/cmake-3.26.4.tar.gz
tar -zxvf cmake-3.26.4.tar.gz
cd cmake-3.26.4
```
3. Build and install CMake
```bash
./bootstrap
make -j$(nproc)
sudo make install
```
**Note:** -j$(nproc) uses all CPU cores to speed up compilation.
4. Verify the version
```bash
cmake --version
```
It should now show something like cmake version 3.26.4.


## References
- ROS1 Distros and target platforms
https://www.ros.org/reps/rep-0003.html#noetic-ninjemys-may-2020-may-2025

- ROS1 Melodic ARM64 build
https://hub.docker.com/r/tiryoh/ros-melodic-desktop/tags

- ROS Docker Hub
https://hub.docker.com/_/ros/

- Install Docker
https://docs.docker.com/engine/install/ubuntu/