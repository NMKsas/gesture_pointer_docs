---
layout: page
title: Getting started
parent: ROS2 release
permalink: /ros2_release/getting_started
nav_order: 2
---

# Prerequisites 

Launch the camera and pose estimation nodes before the gesturing node.

## Camera node 
The repository was developed with Intel RealSense D415 camera and uses ROS wrapper to launch the node

```bash 
# sourcing ROS 
source /opt/ros/humble/setup.bash 

# launching RealSense node
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=True camera_name:=st_cam color_width:=1280 color_height:=720 color_fps:=30 
```

Verify the launch is successful by viewing the camera streams, using e.g., `rqt`

```bash 
# launch rqt to verify the streams are functional 
ros2 run rqt_image_view rqt_image_view
```
Look for streams, 
- `/camera/st_cam/color/image_raw`, 
- `/camera/st_cam/aligned_depth_to_color/image_raw` 

Adjust the camera settings if needed by using `rqt` plugin,   

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
It is important that the depth stream is consistent and has adequate quality. In addition, the upper body (shoulders) of the user must be visible for the camera.

## Pose estimation node 

After launching the camera node, pose estimation can be initialized. First, build the workspace

```bash 
# source ROS2 
source /opt/ros/humble/setup.bash

# build and source the environment
cd ros2env/
colcon build 
source install/setup.bash 
```

After sourcing the environment, run the pose estimation node 
```bash 
# launching the pose estimation
ros2 run pose_keypoint_detector yolo_pose_detector 
```

Verify the keypoints are published by echoing the topic for shoulder, elbow and wrist points 
```bash
ros2 topic echo /pose_keypoints
```
The pixel coordinates (x,y) should reflect the body keypoint positions. 

# Defining the workplane 

To take gesturing tool in use, you need to define a workplane by selecting four corners of the plane. There are two ways to do this:

I. Using RGB-D stream, visually selecting the corners of the desired workplane

II. Using ArUco markers of predefined size 

