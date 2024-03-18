[![Basic Build Workflow](https://github.com/UniBwTAS/continuous_tracking/actions/workflows/basic-build-ci.yaml/badge.svg?branch=master)](https://github.com/UniBwTAS/continuous_tracking/actions/workflows/basic-build-ci.yaml)
[![Publish Docker image](https://github.com/UniBwTAS/continuous_tracking/actions/workflows/publish-docker-image.yaml/badge.svg)](https://github.com/UniBwTAS/continuous_tracking/actions/workflows/publish-docker-image.yaml)

# Low Latency Tracking on Continuous Clustering for Rotating LiDARs

![Continuous Tracking Demo](https://github.com/UniBwTAS/continuous_tracking/blob/master/assets/demo.gif)

## Video:

[![Explanation Video](https://img.youtube.com/vi/dgkrTTSomjA/0.jpg)](https://www.youtube.com/watch?v=dgkrTTSomjA)

## Abstract:

In an intelligent vehicle, it is essential to minimize the latencies in order to be able to react fast. Especially in the
early stages of the perception pipeline, on which many subsequent steps depend, the processing time should be minimized.
One of these fundamental processing steps often includes the instance segmentation of LiDAR point clouds. In a previous
work, we presented an algorithm that significantly reduces the overall latency by processing the incoming point measurements
immediately instead of first accumulating the points for a full revolution. More precisely, the points of a newly incoming
column of the LiDAR range image are immediately clustered to the existing points. Once the LiDAR sensor has rotated
far enough that geometrically no points can be close enough to an existing cluster, this instance segment is published. In
addition to the low latency, this approach has the advantage that there are no problematic discontinuities between the end
of the previous point cloud and the start of the new one. In this work, we present a simple but effective multi-object
tracking algorithm that consistently continues this philosophy and achieves an average latency of only 8 ms and enough
throughput to run in real time. We particularly investigate the association problem since the resulting clusters are no longer
accumulated for a full revolution but arrive in a continuous and out-of-sequence manner. Despite these challenges, we are
capable of keeping pace with other state-of-the-art approaches while significantly reducing latency. Another advantage is that
our method is able to track every object in the environment, regardless of the object class. We are publishing the source
code at https://github.com/UniBwTAS/continuous_tracking.

## Acknowledgement

The authors gratefully acknowledge funding by the Federal Office of Bundeswehr Equipment, Information Technology and
In-Service Support (BAAINBw).

## Overview

- [Examples](#examples)
- [Get Started](#run-it-yourself)
    - [1. Download Sensor Data](#1-download-sensor-data)
    - [2. Setup Environment](#2-setup-environment)
        - [2.1. Option: Docker + GUI (VNC)](#21-option-docker--gui-vnc)
        - [2.2. Option: Locally on Ubuntu 20.04 (Focal) and ROS Noetic](#22-option-locally-on-ubuntu-2004-focal-and-ros-noetic)
    - [3. Run Continuous Tracking](#3-run-continuous-clustering)
- [TODOs](#todos)

# Run it yourself:

## 1. Download Sensor Data

Download the [example rosbag](https://mega.nz/file/7NU11QxQ#-h3AotgPuyCyZaFWPGN0yxfDGNF6YZZM2ppw9QkMxEc) from our VW Touareg test vehicle.

## 2. Setup Environment

### 2.1. Option: Docker + GUI (VNC)

This option is the fastest to set up. However, due to missing hardware acceleration in the VNC Docker container for RVIZ
the rosbag is played at 1/10 speed.

1. Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/)
2. Ensure shell variable `ROSBAG_PATH` is (still) set correctly (see above)
3. Pull and run docker container:

```bash
docker run -d -p 6080:80 -v /dev/shm:/dev/shm -v ${ROSBAG_PATH}:/mnt/rosbags -e ROSBAG_PATH=/mnt/rosbags --name continuous_tracking_demo andreasr30/continuous_tracking_demo:master
```

4. Open your browser on host and enter: http://localhost:6080 (wait a few seconds and retry if it does not load)
5. Open terminal in browser window (click 'Start' -> 'System Tools' -> 'LXTerminal')
6. Continue with step "Run Continuous Tracking" (see below) in the terminal opened in step 2. (There you can use the
   clipboard feature of noVNC; tiny arrow on the left of the screen)

### 2.2. Option: Locally on Ubuntu 20.04 (Focal) and ROS Noetic

```bash
# install ROS (if not already installed)
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/install_ros.sh
bash /tmp/install_ros.sh

# setup ROS workspace (if not already existing)
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_clustering/master/scripts/setup_workspace.sh
bash /tmp/setup_workspace.sh # created at ~/catkin_ws

# switch to your ROS workspace, in our case:
cd ~/catkin_ws/src

# install dependencies and clone repos to current working directory
wget -P /tmp https://raw.githubusercontent.com/UniBwTAS/continuous_tracking/master/scripts/clone_repositories_and_install_dependencies.sh
bash /tmp/clone_repositories_and_install_dependencies.sh
catkin build
```

## 3. Run Continuous Tracking

```bash
# run on VW Touareg rosbag (set the playback speed lower in Rviz (RosbagPanel) if you run in docker due to missing 
# hardware acceleration for graphical output)
roslaunch continuous_tracking continuous_tracking.launch bag_file:=${ROSBAG_PATH}/vw_touareg_example5.bag
```

# TODOs

- Port it to ROS2: planned soon