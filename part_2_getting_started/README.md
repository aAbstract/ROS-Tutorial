# ROS Tutorial Part 2 - Getting Started

### Install Prequiesties

**Operating System:**  
- There are no constraints on the OS as long as docker exists but **Ubuntu LTS** is recommended  

**Docker:**
- Install docker for your OS from this [link](https://docs.docker.com/engine/install/)
- On ubuntu you should add your system user to docker group using this command then reboot the system
```bash
$ sudo usermod -aG docker $USER
```
- Verify docker installation
```bash
$ docker -v

# Docker version 27.3.1, build ce12230
```

**Python3 + rospy:**
- In this guide we will interface with ROS network using python and rospy client library
- If you are using ubuntu, python will be installed by default
- For Windows and Mac users install python from this [link](https://www.python.org/downloads/)
- To install rospy library on ubuntu use this command
```bash
$ sudo apt install python3-rospy
```
- For Windows and Mac users you can follow the conda guide to install rospy in this [link](https://anaconda.org/conda-forge/ros-rospy)

### Notes
- We will be running ROS master and ROS core utilities in a docker container to skip the hassle of manually installing ROS
- If you want to manually install ROS on your system, keep in mind that ROS only works on Ubuntu 20.04 and it reached end of life
- In this guide we will be using the official ROS [docker image](https://hub.docker.com/_/ros/)

### Starting ROS Master
- Create an empty folder
```bash
$ mkdir minesweeper_ros_backend
```
- Create docker-compose.yml file
```bash
$ cd minesweeper_ros_backend && touch docker-compose.yml
```
- Open the file in a text editor and paste this docker-compose yaml script
```yml
services:

  ros:
    # stdin_open: true
    # tty: true
    image: 'ros:noetic-ros-base-focal'
    network_mode: 'host'
    environment:
      - ROS_MASTER_URI=http://<ros_master_machine_ip>:11311/
      - ROS_IP=<ros_master_machine_ip>
    command: 'roscore'

```
- Replace <ros_master_machine_ip> with the ip of the machine running ROS master node
- Run docker-compose.yml using this command
```bash
$ docker compose up -d
# -d: stands for detach, this flag will tell docker to start ROS master container in the background

# Container minesweeper-ros-backend-ros-1  Started 
```
- Verify ROS master container is up and running
```bash
$ docker ps -a

# CONTAINER ID   IMAGE                       COMMAND                  CREATED              STATUS              PORTS     NAMES
# 9a4973a2abdb   ros:noetic-ros-base-focal   "/ros_entrypoint.sh …"   About a minute ago   Up About a minute             minesweeper-ros-backend-ros-1
```

### ROS Hello World - ROS CLI
- Connect to ROS master container shell
```bash
$ docker exec -it minesweeper-ros-backend-ros-1 /bin/bash
# exec /bin/bash: means run /bin/bash program in the docker container which will spawn a shell
# -it: will attach a pseudo-TTY terminal to the spawned bash shell

# root@ubuntu-server:/#
```
- Once connected to the container shell check the file system
```bash
$ ls

# bin  boot  dev  etc  home  lib  lib32  lib64  libx32  media  mnt  opt  proc  root  ros_entrypoint.sh  run  sbin  srv  sys  tmp  usr  var
```
- Setup shell environment
```bash
$ source ros_entrypoint.sh
```
- Start ROS publisher process
```bash
$ rostopic pub /test_topic std_msgs/String "Hello World"
# /test_topic: topic name
# std_msgs/String: message type
# "Hello World": payload

# publishing and latching message. Press ctrl-C to terminate
```
- List created ROS topics
```bash
$ rostopic list

# /rosout
# /rosout_agg
# /test_topic
```
- Start ROS subscriber process in another terminal connected to ROS master container
```bash
$ rostopic echo /test_topic

# data: "Hello World"
# ---
```
