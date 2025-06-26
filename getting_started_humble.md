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

Open another Terminal to verify the keypoints are published. Echo the topic `pose_keypoints` to view values for shoulder, elbow and wrist points 
```bash
# source ROS2 
source /opt/ros/humble/setup.bash

# source workspace to recognize the topic interface 
cd ros2env/
source install/setup.bash

ros2 topic echo /pose_keypoints

# example print for the message 
header:
  stamp:
    sec: 1750930022
    nanosec: 268075999
  frame_id: st_cam_color_optical_frame
pose_id: 0
keypoint_list:
- kpt_name: l_sho
  conf: 0.9052868485450745
  x: 478
  y: 62
- kpt_name: r_sho
  conf: 0.9528919458389282
  x: 343
  y: 53
...
```

# Defining the workplane 

To take gesturing tool in use, you need to define a workplane by selecting four corners of the plane. There are two ways to do this:

I. Using RGB-D stream, visually selecting the corners of the desired workplane

II. Using ArUco markers of predefined size 

## Setup constants for cache 

Before running any code, peek at `constants.py` file, and update the default path to your workspace - the files are saved by default under `/data` directory in the package. 
You may switch the cache on and off by changing the boolean parameter (True / False). 
If cache is enabled, the corners and mask defined are saved and used during the next launch automatically.

```python 
# generic constants 
DEFAULT_PATH = "/<your_workspace>/src/gesture_pointer/data"
CACHE_ENABLED = True                
CORNER_CACHE_FILE = 'corner_cache'  # default filename
MASK_CACHE_FILE = 'mask_cache'      # default filename 
```

## I. Using RGB-D stream 

GUI method is used by default if a corner file **is not** provided. If you want to use the GUI method despite the existing corner file, leave the corner file name empty in `constants.py` file

```python 
CORNERS_CSV_FILE = '' 
```

Run the gesturing node: 

```bash
# source ROS2 and the workspace 
source /opt/ros/humble/setup.bash
cd ros2env
source install/setup.bash

# launching the node 
ros2 run gesture_pointer gesture_pointer
```

Soon after running the node, a depth-image of the camera stream should appear as a pop-up window. Proceed to click the corners of the desired workspace in the requested clockwise order, starting from lower left corner. Once you are finished, press `Q` to close the pop-up window. After this, the gesture node is set up and running.

<iframe width="560" height="315" src="https://www.youtube.com/embed/NUYPBbpYvpg?si=uNaocCbzMTSnjL8Q" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The current implementation is not flexible in terms of changing the corner order. It is however possible by tinkering few constants in the code (instructions and improvements on this part are under development).

**Note**: The plane is defined based on the first three corners. Just as seen in the video, it doesn't matter if the node gets bad, occluded depth readings for the last, fourth corner.

To verify that the node is running and everything works, check the gesturing stream by launching e.g., 

```bash
ros2 run rqt_image_view rqt_image_view
```

You should be able to see topic `gesture_projection` that streams the gesturing visualizations.

## II. Using ArUco markers 

The user can define plane corners with ArUco markers of pre-defined size using RGB stream. The module includes a separate launch file, which uses [`aruco_ros`](https://github.com/pal-robotics/aruco_ros.git) package to fetch and save the four marker positions into a `.csv` file.

![Defining the plane with ArUco markers](../assets/images/aruco_method.png)

The repository provides two [ArUco grids of size 80x60cm](https://github.com/NMKsas/gesture_pointer/tree/7aeae229d7f6e526c57093ee2052c4f51c1a4380/testing_boards), with 8x8cm ArUco markers positioned at the corners. 
 
### 1. Place the markers on the workspace corners

Print four ArUco markers: measure the sizes of the markers and identify the IDs. If you use the provided [grid board](https://github.com/NMKsas/gesture_pointer/blob/7aeae229d7f6e526c57093ee2052c4f51c1a4380/testing_boards/grid_board_80x60c   m.pdf), markers with IDs 100-103 from the original ArUco collection are used. 

### 2. Replace the values `constants.py` file

Replace the constants for data collection in `constants.py`

```python 
# ArUco data collection constants 
CAMERA_TF_FRAME = 'st_cam_color_optical_frame'
CAMERA_RGB_TOPIC = '/camera/st_cam/color/image_raw'
CAMERA_INFO_TOPIC = '/camera/st_cam/color/camera_info'
CORNERS_CSV_FILE = 'corners_mean.csv'
CORNER_MARKER_SIZE = 0.08 # m
MARKERS = [100,101,102,103] 
```

### 3. Launch the collection node 

After filling in the details, you can launch the aruco collection node. 

```bash 
ros2 launch gesture_pointer aruco_data_collection_launch.py
```

The node will launch detection for each four markers, and attempt to collect 20 samples for each marker position. The mean of the samples is saved into `.csv` file. The progress is visible on Terminal screen, and the node shuts down automatically once the collection is complete. 

**Debugging**

Sometimes the ArUcos are not detected correctly, due to RGB stream quality. If the collection fails, launch a single marker file 

```bash
# source ROS and workspace 
source /opt/ros/humble/setup.bash 
cd ros2env
source install/setup.bash 

# launch a single marker 
ros2 launch gesture_pointer single_marker_launch.py 
```
It is possible to inspect the debugging stream via `RViz` or `rqt`.
```bash
ros2 run rqt_image_view rqt_image_view
```

![Debugging ArUco detection RGB stream](../assets/images/aruco_debug_ros2.png)

Inspect the `/aruco_single/result` topic. Verify that the corner markers are detected and adjust the camera settings if needed, using `rqt` plugin.

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
Once the detections are stable on the stream, relaunch `aruco_data_collection_launch.py`. Verify that the file has appeared to the directory defined in `constants.py`. 

### 4. Launch the gesturing node 

After the positions are successfully collected, the launched gesturing node uses the four ArUco corners as the workplane. 

```bash
# source ROS2 and workspace 
source /opt/ros/humble/setup.bash 
cd ros2env/
source install/setup.bash

# launching the node 
ros2 run gesture_pointer gesture_pointer
```

