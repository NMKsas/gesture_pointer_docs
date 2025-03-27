---
layout: page
title: Getting started
permalink: /getting_started/
nav_order: 2
---

# Prerequisites 

To launch the gesturing tool, you need to launch the camera and pose estimation nodes. The repository was developed with Intel RealSense D415 camera and uses ROS wrapper, to launch the node

```bash 
# sourcing ROS 
source /opt/ros/noetic/setup.bash 

# launching RealSense node
roslaunch realsense2_camera rs_camera.launch align_depth:=True color_width:=1280 camera:=<camera_name> color_height:=720 color_fps:=30 initial_reset:=True
```

After launching the camera node, pose estimation can be initialized. If you followed the installation instructions and placed the `pose_estimation_node.py` under `gesture_pointer` directory, run the node as follows

```bash 
# sourcing ROS 
source /opt/ros/noetic/setup.bash
 
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

If you can see the camera streams for each topic, you are good to go. 

# Defining the workplane 

To take gesturing tool in use, you need to define a workplane by selecting four corners of the plane. There are two ways to do this:

1. Using RGB-D stream, visually selecting the corners of the desired workplane 
2. Using ArUco markers of predefined size 
                                                                                    
## 1. Using RGB-D stream 

This method is used by default, and launches if corner file is not provided. Run the gesturing node: 

```bash
# sourcing the package 
source devel/setup.bash 

# launching the node 
rosrun gesture_pointer gesture_pointer
```

Soon after running the node, a depth-image of the camera stream should appear as a pop-up window. Proceeding to click the corners of the desired workspace in the requested clockwise order, starting from upper left corner. Once you are finished, press `Q` to close the pop-up window. After this, the gesture node is set up and running.

<video width="640" height="360" controls>
  <source src="https://github.com/NMKsas/gesture_pointer_docs/blob/main/demo_videos/defining_plane_rgb-d.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

The current implementation is not flexible in terms of changing the corner order. It is possible quite easily by tinkering few constants in the code (instructions and improvements on this part are under development).

**Note**: The plane is defined based on the first three corners. Just as seen in the video, it doesn't matter if the node gets bad, occluded depth readings for the last, fourth corner.

To verify that the node is running and everything works, check the gesturing stream by launching e.g., 

```bash
rosrun rqt_image_view rqt_image_view
```

You should be able to see topic `gesture_projection` that streams the tool in action.

## 2. Using ArUco markers 
