---
layout: page
title: About
permalink: /
nav_order: 0
---


This is the documentation for GesturePointer tool. The tool uses RGB-D stream and [OpenPose](https://arxiv.org/abs/1812.08008)-based pose estimation node developed in [OpenDR project](https://github.com/opendr-eu/opendr) to localize and publish pointed targets as ROS topics.

<iframe width="560" height="315" src="https://www.youtube.com/embed/6pz6xVdxndg?si=_xCGAT41A2rG3sZX" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

The tool was developed using ROS1 Noetic distribution; ROS2 Humble draft implementation exists but is yet to be released. The original work was developed for Intel RealSense D415 camera.

Author: Noora Sassali, [`@NMKsas`](https://github.com/NMKsas) 

# Overview

Two ROS nodes need to be launched in prior to the gesturing node itself: a camera node and a 2D pose estimation node.
The inputs and outputs are presented in the below diagram. 


![Dataflow diagram](assets/images/dataflow_diagram.png)

The camera node has to publish three topics: 
- RGB stream
- aligned depth-to-color (D) stream
- camera intrinsics

The RGB stream is directly used by a 2D pose estimation node, which detects human body keypoints as 2D image coordinates. The keypoints are subscribed by Gesturing node, which utilizes the keypoints for shoulders and wrists to detect the pointed targets.

The gesturing node has an instance of `CameraSubscriber` class, which subscribes RGB-D stream and camera intrinsics. The class interface includes getter functions for the latest RGB and depth frames, and functions for coordinate deprojection (from 2D to 3D) and projection (from 3D to 2D). 

The gesturing node outputs three different ROS topics: 
- a localized point on plane
- a projection stream
- visualization markers. 

The projection stream shows the borders of the user-defined workspace and visual cues for the localized gestures. The stream can be used to direct the gestures better and troubleshoot possible issues with gesturing. The localized point consists of a timestamp and the pointed coordinate with respect to the camera coordinate frame or workplane frame. The visualization markers for `RViz` create spherical points for the localized points on plane. 

**Note: The documentation is still a work in progress.**
