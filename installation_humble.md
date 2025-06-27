---
layout: page
title: Installation
parent: ROS2 release
permalink: /ros2_release/installation
nav_order: 1
---

- TOC 
{:toc}

# Installation  

The gesturing module for ROS2 is developed for [Humble Hawksbill](https://docs.ros.org/en/humble/index.html) distribution. Install ROS2 as instructed in the [documentation](https://docs.ros.org/en/humble/Installation.html). 

Start the use of gesturing tool by cloning the repository,

```bash
git clone https://github.com/NMKsas/gesture_pointer.git --branch ros2_humble
```

Make sure ROS2 dependencies are installed by running `rosdep` 

```bash
# run once, if rosdep is not initialized yet
sudo rosdep init 

# update rosdep database
rosdep update 

# install ROS package dependencies
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## Python dependencies
{: .no_toc }
All the `python` dependencies are listed in the `requirements.txt` file in the repository root directory.
```bash
# Install Python dependencies
pip install -r requirements.txt
```

## Intel RealSense camera and ROS wrapper
{: .no_toc }
The module was developed for **Intel RealSense D415**, and supports D400 product family, given `aligned_depth_to_color` stream can be extracted. The current implementation is dependent on `pyrealsense2` Python package (installed in the previous step). Install the ROS wrapper as instructed on [Intel RealSense wrapper GitHub page](https://github.com/IntelRealSense/realsense-ros.git). 

Despite the RealSense dependency, you are free to adapt the tool for other cameras, by modifying `CameraSubscriber` submodule accordingly.

## (Aruco ROS library) 
{: .no_toc }
If you want to use ArUco markers to establish the workplane, the repository provides ArUco node launchers and simple script for collecting pose data using [`aruco_ros`](https://github.com/pal-robotics/aruco_ros.git) package. The package is mentioned in `package.xml` file, and should install when you run `rosdep` on the top directory. However, the installation is not mandatory, should you use the graphical user interface based method (RGB-D) to define the workplane - feel free to comment the dependency out!

# Docker Container 

Prerequisite: Install [Docker](https://docs.docker.com/engine/)

Clone the repository,
```bash
git clone https://github.com/NMKsas/gesture_pointer.git --branch ros2_humble
```

The repository includes `docker-compose.yaml` and `Dockerfile` for using the repository with a docker container. The files are constructed such, that the dependencies are installed automatically. 
## Volume binds 
{: .no_toc }
Define the directory binds between the local workspace and the container workspace. By default, Dockerfile will create a `/up/ros2env/` directory and bind the local, cloned repository to the container.

```docker
# docker_compose.yaml     
    volumes: 
      - ./ros2env/src:/up/ros2env/src   # change the local directory if necessary 
```
In addition, you may need to change the video directories for the used camera. Check your local system for `/dev/video*` directories and modify the file accordingly. For visualization purposes (`RViz`, `rqt`, `cv2`), you need to give certain rights for screen sharing, before opening docker terminals 

```bash 
# necessary for using e.g., cv2.imshow(), RViz, ...
# ...beware the security risks! (Read more: https://linux.die.net/man/1/xhost) 
xhost +local:<username>

# after use, remove the access rights 
xhost -local:<username> 
```

Launch the container at root directory of the repository 
```bash 
sudo docker compose up 
```

Launch interactive terminals for the container: 
```bash
sudo docker exec -it gp_container /bin/bash 
```

And you are all set...!

