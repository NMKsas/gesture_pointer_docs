---
layout: page
title: Getting started
parent: ROS1 release
permalink: /ros1_release/getting_started
nav_order: 2
---

# Prerequisites 

To launch the gesturing tool, you need to launch the camera and pose estimation nodes. The repository was developed with Intel RealSense D415 camera and uses ROS wrapper, to launch the node

```bash 
# sourcing ROS 
source /opt/ros/noetic/setup.bash 

# launching RealSense node
roslaunch realsense2_camera rs_camera.launch align_depth:=True color_width:=1280 camera:=st_cam color_height:=720 color_fps:=30 
```

After launching the camera node, pose estimation can be initialized. If you followed the installation instructions and placed the `pose_estimation_node.py` under `gesture_pointer` directory, run the node as follows

```bash 
# sourcing ROS 
source /opt/ros/noetic/setup.bash

# source the workspace that was previously built with catkin_make 
cd <your_workspace> 
source devel/setup.bash
 
# launching the pose estimation
rosrun gesture_pointer pose_estimation_node.py -i /<camera_name>/color/image_raw 
```

Verify the launch is successful by viewing the camera and pose estimation streams, using e.g., `rqt`

```bash 
# launch rqt to verify the streams are functional 
rosrun rqt_image_view rqt_image_view
```
Look for streams, 
- `/<camera_name>/color/image_raw`, 
- `/<camera_name>/aligned_depth_to_color/image_raw` 
- `/opendr/poses`

If you can see the camera streams for each topic, the nodes are launched properly. 

Adjust the camera settings if needed by using `rqt` plugin,   

```bash
rosrun rqt_reconfigure rqt_reconfigure
```
It is important that the depth stream is consistent and has adequate quality. In addition, the upper body (shoulders) of the user must be visible for the camera. 

# Defining the workplane 

To take gesturing tool in use, you need to define a workplane by selecting four corners of the plane. There are two ways to do this:

I. Using RGB-D stream, visually selecting the corners of the desired workplane

II. Using ArUco markers of predefined size 

## Setup constants for cache 

Before running any code, peek at `constants.py` file, and update the default path to your workspace - the files are saved by default under `/data` directory in the package. 
You may switch the cache on and off by changing the boolean parameter (True / False). 
If cache is enabled, the corners and mask defined using either of the methods are saved and used during the next launch automatically.

```python 
# generic constants 
DEFAULT_PATH = "/<your_workspace>/src/gesture_pointer/data"
CACHE_ENABLED = True                
CORNER_CACHE_FILE = 'corner_cache'  # default filename
MASK_CACHE_FILE = 'mask_cache'      # default filename 
```

## I. Using RGB-D stream 

GUI method is used by default if corner file **is not** provided. If you want to use the GUI method despite the existing corner file, leave the corner file name empty in `constants.py` file

```python 
CORNERS_CSV_FILE = '' 
```

Run the gesturing node: 

```bash
# sourcing the package 
source devel/setup.bash 

# launching the node 
rosrun gesture_pointer gesture_pointer
```

Soon after running the node, a depth-image of the camera stream should appear as a pop-up window. Proceed to click the corners of the desired workspace in the requested clockwise order, starting from lower left corner. Once you are finished, press `Q` to close the pop-up window. After this, the gesture node is set up and running.

<iframe width="560" height="315" src="https://www.youtube.com/embed/NUYPBbpYvpg?si=uNaocCbzMTSnjL8Q" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The current implementation is not flexible in terms of changing the corner order. It is however possible by tinkering few constants in the code (instructions and improvements on this part are under development).

**Note**: The plane is defined based on the first three corners. Just as seen in the video, it doesn't matter if the node gets bad, occluded depth readings for the last, fourth corner.

To verify that the node is running and everything works, check the gesturing stream by launching e.g., 

```bash
rosrun rqt_image_view rqt_image_view
```

You should be able to see topic `gesture_projection` that streams the tool in action.

## II. Using ArUco markers 

The user can define plane corners with ArUco markers of pre-defined size using RGB stream. The module includes a separate `.roslaunch` file, which uses [`aruco_ros`](https://github.com/pal-robotics/aruco_ros.git) package to fetch and save the four marker positions into a `.csv` file.

![Defining the plane with ArUco markers](../assets/images/aruco_method.png)

The repository provides two [ArUco grids of size 80x60cm](https://github.com/NMKsas/gesture_pointer/tree/7aeae229d7f6e526c57093ee2052c4f51c1a4380/testing_boards), with 8x8cm ArUco markers positioned at the corners. 
 
### 1. Place the markers on the workspace corners

Print four ArUco markers: measure the sizes of the markers and identify the IDs. If you use the provided [grid board](https://github.com/NMKsas/gesture_pointer/blob/7aeae229d7f6e526c57093ee2052c4f51c1a4380/testing_boards/grid_board_80x60cm.pdf), markers with IDs 100-103 from the original ArUco collection are used. 

### 2. Replace the values `aruco_corner_collection.launch` file

To collect the ArUco marker positions, replace the values for arguments in `aruco_corner_collection.launch` file. 

```xml
<launch>

    <!-- ArUco marker size in cm -->
    <arg name="corner_marker_size" value="0.08"/> 
    <!-- the reference frame for the RGB stream -->
    <arg name="reference_frame"    value="st_cam_color_optical_frame"/> 

    <!-- Corner Markers -->
    <include file="$(find gesture_pointer)/launch/single_marker.launch">
        <!-- marker ID -->
        <arg name="marker_id"           value="103"/> 
        <arg name="marker_size"         value="$(arg corner_marker_size)"/>
        <arg name="reference_frame"     value="$(arg reference_frame)"/>
        <!-- name for the marker frame; add the corresponding marker ID postfix -->
        <arg name="marker_frame"        value="aruco_103"/>
    
``` 

### 3. Replace the values in `single_marker.launch` file 

Replace the argument values to match your camera setup. 

```xml
<launch>

    <arg name="marker_id" default=""/>
    <arg name="marker_size" default=""/>
    <arg name="reference_frame" default=""/>
    <arg name="marker_frame" default=""/>
    
    <!-- Replace the values for these parameters with your own camera setup -->
    <arg name="camera_frame"            value="st_cam_color_optical_frame"/> 
    <arg name="camera_info_topic"       value="st_cam/color/camera_info"/>
    <arg name="input_image_topic"       value="st_cam/color/image_raw"/>
    <arg name="is_image_rectified"      value="True"/>     
    <!-- ################################################################## -->

```

### 4. Replace the values in `constants.py` file

The `constants.py` file needs to be updated to correspond the used setup. Fill in the camera coordinate frame (`/tf` frame for the aligned depth stream), the default name for the saved corner file and the Ids of the used ArUco markers. 

```python 
# ArUco data collection constants 
CAMERA_TF_FRAME = 'st_cam_color_optical_frame'
CORNERS_CSV_FILE = 'corners_mean.csv'
MARKERS = [100,101,102,103] 
``` 

### 5. Launch the collection node 

After filling in the details, you can launch the aruco collection node. 

```bash 
roslaunch gesture_pointer aruco_data_collection 
```

The node will launch detection for each four markers, and attempt to collect 20 samples for each marker position. The mean of the samples is saved into `.csv` file. The progress is visible on Terminal screen, and the node shuts down automatically once the collection is complete. 

**Debugging**

Sometimes the ArUcos are not detected correctly, due to RGB stream quality. If the collection fails, set the `required` flag from the `aruco_corner_collection.launch` file false: 

```xml
    <!-- Collect the data -->
    <node pkg="gesture_pointer" type="aruco_data_collection" name="aruco_workspace"
          output="screen" required="false"> 
    </node>
```
Normally `required="true"` will automatically close each of the single marker detection entities, once the data collection node is ran. `required="false"` will leave the marker nodes up even if the collection fails, and it is possible to inspect the debugging stream via `RViz` or `rqt`. Each marker publishes `/result` stream.

![Debugging ArUco detection RGB stream](../assets/images/aruco_debug.png)

Observe the detections in `/result` stream and adjust the camera settings if needed, using `rqt` plugin.

```bash
rosrun rqt_reconfigure rqt_reconfigure
```
Once the detections are stable on the stream, set the `required` flag back to `True` and relaunch the collection node.

### 6. Launch the gesturing node 

After the positions are successfully collected, the launched gesturing node uses the four ArUco corners as the workplane. 

```bash
# sourcing the package 
source devel/setup.bash 

# launching the node 
rosrun gesture_pointer gesture_pointer
```

