# Guidance-System
Building automated guidance system using robot-mounted or handheld zed-mini.
![Top 20 percent](https://github.com/ghost-60/Guidance-System/blob/master/sample/sample.png "A very good girl")
## Current Status
- Using Semantic SLAM through ROS nodes to improve localization in feature-less environments.
- Semantic segmentation network is using ResNet18dilated + C1_deepsup architecture.
- Rtabmap ROS implementaion is integrated with move_base package for path planning.
- Using the segmented images and depth map, text/voice based commands are being generated.

### Added Feature:
- Given an a priori map, localize using landmarks such as doors, walls, etc.
- Using particle filtering to localize in apriori map.
- Integrated with ORB-SLAM to obtain state of particles and actual object.

## Setup
- Setup the Jetson Xavier using the "SDKManager" and install all jetson SDK components. https://www.stereolabs.com/developers/nvidia-jetson/
- Install ROS melodic http://wiki.ros.org/melodic/Installation/Ubuntu

- Install ZED-ROS-Wrapper https://github.com/stereolabs/zed-ros-wrapper

- Install cv_bridge from source. https://github.com/ros-perception/vision_opencv/tree/melodic/cv_bridge . Set the following variables while calling cmake (Note: Path and file name may differ)
 ```
 $ cmake 
-DPYTHON_EXECUTABLE=/usr/bin/python3
-DPYTHON_INCLUDE_DIR=/usr/include/python3.6m 
-DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so ../
```

- Install python packages - pytorch, scipy(v1.1.0), numpy, torchvision, yacs, tqdm
 (Note: Newer versions of scipy have some issue)

- Build the Rtabmap package https://github.com/introlab/rtabmap/wiki/Installation

- Build Rtabmap_ros
```
$ catkin_make
```
## How to run

1. Run semantic segmentaion
```
$ cd /path/to/semantic/segmentation
$ python sample_Resnet18.py
```
2. Run the rtabmap and zed camera.
```
$ roslaunch visguide visguide.launch
```

3. To run the rtabmap and zed camera seperately:
```
$ roslaunch visguide rtabmap.launch
$ roslaunch visguide zedm.launch
```
4. To run particle filtering in apriori map:
```
$ roslaunch visguide landmark.launch
```

5. For visualisation
```
$ rviz
```

## Recording data using zed
(Note: imu data is not stored in svo file. Use rosbag to collect imu data)
- Use rviz to view live image.
```
$ roscore
$ roslaunch zed_wrapper zedm.launch
$ rosservice call /zed/zed_node/start_svo_recording svo_filename:= /path/to/svo/file_name.svo
$ rosservice call /zed/zed_node/stop_svo_recording
```

