---
layout: page
title: Installation
permalink: /installation/
nav_order: 1
---

# Installation  

The tool is developed for [ROS1 Noetic Ninjemys](https://wiki.ros.org/noetic) distribution. Install ROS1 as instructed in the [documentation](https://wiki.ros.org/noetic/Installation). 

Start the use of gesturing tool by cloning the repository,

```bash
git clone ...
```

Go to the top directory of your catkin workspace and install the package dependencies,

```bash
# update rosdep database
rosdep update --rosdistro noetic  

# install packages
rosdep install --from-paths src --ignore-src -r -y
```
# Dependencies 

## Intel RealSense camera and ROS wrapper 
The current implementation was developed for **Intel RealSense D415**, and supports D400 product family, given `aligned_depth_to_color` stream can be extracted. The current implementation is dependent on `pyrealsense2` package,

```bash
# install the RealSense python library   
pip install pyrealsense2
```
Despite the RealSense dependency, you are free to adapt the tool for other cameras, by modifying `CameraSubscriber` submodule accordingly. 

Should you use RealSense camera, install the ROS wrapper as instructed on [Intel RealSense wrapper GitHub page](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy). 

## OpenDR Pose estimation

The original work uses [pose estimation node](https://github.com/opendr-eu/opendr/tree/master/projects/opendr_ws/src/opendr_perception#pose-estimation-ros-node) from OpenDR project. For using the node, follow the [installation instructions](https://github.com/opendr-eu/opend
r/blob/master/docs/reference/installation.md) provided by the project to install the whole OpenDR environment, **or** install only the relevant files by following these steps: 

1. Install OpenDR pose estimation dependencies 

    ```bash
    # dependencies for pose estimation 
    pip install opendr-toolkit-engine
    pip install opendr-toolkit-pose-estimation
    ```
    
    In case you get a warning from `protobuf` version requirement (<=3.20), update the package to a patched version (`3.20` has vulnerabilities)

    ```bash
    pip install protobuf==3.20.2
    ``` 

2. Copy the pose estimation node to your workspace 

    The file for pose estimation node can be found in [OpenDR project workspace](https://github.com/opendr-eu/opendr/blob/master/projects/opendr_ws/src/opendr_perception/scripts/pose_estimation_node.py). Copy the file to your workspace, e.g., under the `<your_workspace>/src/gesture_pointer/src/gesture_pointer/` directory. 

    Make sure the file is executable,
    ```bash
    chmod +x pose_estimation_node.py
    ```

3. Download `opendr_bridge` 

    Pose estimation node has some dependencies on another OpenDR project package, `opendr_bridge`. If you want to avoid installing OpenDR environment as a whole, only download [this particular directory](https://github.com/opendr-eu/opendr/tree/master/projects/opendr_ws/src/opendr_bridge). Place the package to your workspace (`<your_workspace>/src/`) alongside `gesture_pointer` and `snap_to_target` folders.

    In addition, download the module `hri_msgs`, which is missing in the `opendr_bridge` package dependencies. 
      
    ```bash
    apt-get install ros-noetic-hri-msgs
    ```

4. Build the workspace

    Make sure all the dependencies for `pose_estimation_node.py` are installed.  

    ```bash 
    # navigate to your workspace
    cd <your_workspace>

    # run just in case some entries are missing
    rosdep install --from-paths src --ignore-src -r -y

    # source ros 
    source /opt/ros/noetic/setup.bash

    # build the workspace
    catkin_make
    ```
    
And you are all set! 

## (Aruco ROS library) 

If you want to use ArUco markers to establish the workplane, the repository provides ArUco node launchers and simple script for collecting pose data using [`aruco_ros`](https://github.com/pal-robotics/aruco_ros.git) package. The package is mentioned in `package.xml` file, and should install when you run `rosdep` on the top directory. However, the installation is not mandatory, should you use the graphical user interface based method (RGB-D) to define the workplane - feel free to comment the dependency out!