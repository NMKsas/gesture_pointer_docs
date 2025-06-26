---
layout: page
title: Installation
parent: ROS2 release
permalink: /ros2_release/installation
nav_order: 2
---

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
# Dependencies 

## Python dependencies 

All the `python` dependencies are listed in the `requirements.txt` file in the repository root directory.
```bash
# Install Python dependencies
pip install -r requirements.txt
```

## Intel RealSense camera and ROS wrapper 
The module was developed for **Intel RealSense D415**, and supports D400 product family, given `aligned_depth_to_color` stream can be extracted. The current implementation is dependent on `pyrealsense2` Python package (installed in the previous step). Install the ROS wrapper as instructed on [Intel RealSense wrapper GitHub page](https://github.com/IntelRealSense/realsense-ros.git). 

Despite the RealSense dependency, you are free to adapt the tool for other cameras, by modifying `CameraSubscriber` submodule accordingly.

## (Aruco ROS library) 

If you want to use ArUco markers to establish the workplane, the repository provides ArUco node launchers and simple script for collecting pose data using [`aruco_ros`](https://github.com/pal-robotics/aruco_ros.git) package. The package is mentioned in `package.xml` file, and should install when you run `rosdep` on the top directory. However, the installation is not mandatory, should you use the graphical user interface based method (RGB-D) to define the workplane - feel free to comment the dependency out!