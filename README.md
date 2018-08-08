# KinectBodyTracker

## Description
This Visual Studio project is adapted from an example of Kinect SDK 2.0 named BodyBasics. It features:
1. Extracting the position of the spine base of the subject;
1. Calculating the linear and angular velocity commands for a differential drive robot to follow the subject.
1. Transmiting the command to a computer running ROS via rosserial_windows.


## Prerequisite
1. Windows 10
1. Kinect SDK 2.0
1. Visual Studio 2017

## Cloning

Since it contains a submodule, please use option "--recurse-submodule" to clone to a local directory.
```
git clone --recurse-submodule https://github.com/zchenpds/KinectBodyTracker
```

## Output File Format
1. Kinect Timestamp (mSecs)
1. Odroid Timestamp (uSecs)
1. Time that has elapsed since the receipt of the most recent Odroid timestamp (mSecs)
1. Left knee x coordinate (meters, relative to Camera frame)
1. Left knee y coordinate (meters, relative to Camera frame)
1. Left knee z coordinate (meters, relative to Camera frame)
1. Left ankle x coordinate (meters, relative to Camera frame)
1. Left ankle y coordinate (meters, relative to Camera frame)
1. Left ankle z coordinate (meters, relative to Camera frame)
1. Left foot x coordinate (meters, relative to Camera frame)
1. Left foot y coordinate (meters, relative to Camera frame)
1. Left foot z coordinate (meters, relative to Camera frame)
1. Right knee x coordinate (meters, relative to Camera frame)
1. Right knee y coordinate (meters, relative to Camera frame)
1. Right knee z coordinate (meters, relative to Camera frame)
1. Right ankle x coordinate (meters, relative to Camera frame)
1. Right ankle y coordinate (meters, relative to Camera frame)
1. Right ankle z coordinate (meters, relative to Camera frame)
1. Right foot x coordinate (meters, relative to Camera frame)
1. Right foot y coordinate (meters, relative to Camera frame)
1. Right foot z coordinate (meters, relative to Camera frame)
1. Robot x coordinate (meters, relative to initial pose)
1. Robot y coordinate (meters, relative to initial pose)
1. Robot orientation (radians, relative to initial pose)
1. Robot linear velocity (meters/sec)
1. Robot angular velocity (radians/sec)