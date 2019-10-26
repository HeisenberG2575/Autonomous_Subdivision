# Autonomous Subsystems, IIT-B Mars Rover Team
The repository for autonomous subsystems codes for Mars Rover Project, IIT-Bombay

## Repository contents 

* [GPS_IMU_app](./GPS_IMU_app): Contains code for android app made to publish GPS and IMU sensor readings of an android phone(should have a magnetometer) as rostopics `\LatLon` and `\IMU`
* [Mobility Workspace](./mobility_ws): Contains code for closed loop steer of rover for level 3 autonomous implementation
* [Steer-Drive Switch](./steer_drive_switch.py): **Python2.7** code for shifting between steer and drive modes
* [Ball_Detection](./Ball_Detection): Contains code for ball detection. Also has weights for supervised ball detection
---
## Idetified work points

- [x] Ball detection using GMM
- [ ] Ball detection using CNN (in correspondence with Vito)
- [ ] Illumination correction
- [ ] Stereo_image_proc ros package testing
- [ ] GUI integration with python

---
## Laserscan parameters for kinect2

Use `iai_kinect2` package to obtain depth map. Then, use `depthimage_to_laserscan` package to convert depth image to laserscan. We obtained the following parameters in the laser scan message.

* Scanning range `-0.6 radians to 0.6 radians`, `0.45 meters to 10 meters`
* Scanning precision (increment angle) `0.0024 radians (0.14 degrees)`
* Time between scans `0.033 seconds`

Scan creates a polar map which indicates the obstacles in the 2d plane of the sensor. Obstacles are indicated by `range and theta`.

After connecting Kinect2, run -
```
roslaunch kinect2_bridge kinect2_bridge.launch
rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/kinect2/sd/image_depth
roslaunch obstacle_detector obstacle_detec.launch
rviz
```  
---

## YDLIDAR ROS package

ROS node and test application for YDLIDAR.

Scan creates a polar map which indicates the obstacles in the 2d plane of the sensor. Obstacles are indicated by `range and theta`.

### Procedure for installation

1) Clone [this](https://github.com/EAIBOT/ydlidar.git) project to your catkin's workspace src folder
2) Run `catkin_make` to build ydlidar_node and ydlidar_client
3) Run these commands -

```
roscd ydlidar/startup
sudo chmod 777 ./*
`sudo sh initenv.sh
```

After connecting ydlidar -

a) To view output in rviz : 

`roslaunch ydlidar lidar_view.launch`

**OR**

b) To view output in terminal :

`roslaunch ydlidar lidar.launch`

Open another terminal, Run : 

```
source devel/setup.bash
rosrun ydlidar ydlidar_client
``` 

