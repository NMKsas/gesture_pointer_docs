---
layout: page
title: Installation
parent: ROS1 release
permalink: /ros1_release/installation
nav_order: 1
---
- TOC 
{:toc}

# Installation  

The tool is developed for [ROS1 Noetic Ninjemys](https://wiki.ros.org/noetic) distribution. Install ROS1 as instructed in the [documentation](https://wiki.ros.org/noetic/Installation). 

Start the use of gesturing tool by cloning the repository,

```bash
git clone https://github.com/NMKsas/gesture_pointer.git --branch ros1_noetic
```

Go to the top directory of your catkin workspace and install the package dependencies,

```bash
# update rosdep database
rosdep update --rosdistro noetic  

# install packages
rosdep install --from-paths src --ignore-src -r -y
```

## Intel RealSense camera and ROS wrapper 
{: .no_toc }
The module was developed for **Intel RealSense D415**, and supports D400 product family, given `aligned_depth_to_color` stream can be extracted. The current implementation is dependent on `pyrealsense2` package,

```bash
# install the RealSense python library   
pip install pyrealsense2
```
Despite the RealSense dependency, you are free to adapt the tool for other cameras, by modifying `CameraSubscriber` submodule accordingly. 

Should you use RealSense camera, install the ROS wrapper as instructed on [Intel RealSense wrapper GitHub page](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy). 

## OpenDR Pose estimation
{: .no_toc }
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
{: .no_toc }
If you want to use ArUco markers to establish the workplane, the repository provides ArUco node launchers and simple script for collecting pose data using [`aruco_ros`](https://github.com/pal-robotics/aruco_ros.git) package. The package is mentioned in `package.xml` file, and should install when you run `rosdep` on the top directory. However, the installation is not mandatory, should you use the graphical user interface based method (RGB-D) to define the workplane - feel free to comment the dependency out!


# Docker Container 

Prerequisite: Install [Docker](https://docs.docker.com/engine/)

Clone the repository,
```bash
git clone https://github.com/NMKsas/gesture_pointer.git --branch ros1_noetic
```

The repository includes `docker-compose.yaml` and `Dockerfile` for using the repository with a docker container. The files are constructed such, that the dependencies are installed automatically. 
## Volume binds
{: .no_toc }
Define the directory binds between the local workspace and the container workspace. By default, Dockerfile will create a `/up/ros1env/` directory and bind the local, cloned repository to the container.

```docker
# docker_compose.yaml     
    volumes: 
      - ./ros1env/src:/up/ros1env/src   # change the local directory if necessary 
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
