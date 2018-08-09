# KinectBodyTracker

## Description
This Visual Studio project is adapted from an example of Kinect SDK 2.0 named BodyBasics. It features:
1. Extracting the position of the spine base of the subject;
1. Calculating the linear and angular velocity commands for a differential drive robot to follow the subject;
1. Transmitting the commands to a P3AT robot by using Aria SDK;
1. Receiving UDP synchronization packets from an Odroid computer, one packet per second.


## Prerequisite
1. Windows 10
1. Kinect SDK 2.0
1. Visual Studio 2017 (Version 15.7.4 or above)
1. 32-bit version of ARIA SDK [here](http://robots.mobilerobots.com/wiki/ARIA)

## Cloning
Since it contains a submodule, please use option "--recurse-submodule" to clone to a local directory.
```
git clone --recurse-submodule https://github.com/zchenpds/KinectBodyTracker
```

## Compiling the source
1. Before compiling the source, you might need to retarget the solution if you have a higher version of Windows SDK. To do this, right click on your solution in the Solution Explorer, and then choose "Retarget solution". 
1. Once the source code is compiled, a `.exe` file will be generated in `.\Release` in case of Release mode, or in `.\Debug` in case of Debug mode. Copy either `(Aria_Directory)\bin\AriaVC15.dll` to `.\Release` or `(Aria_Directory)\bin\AriaDebugVC15.dll` to `.\Debug` before running any executable.


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