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
