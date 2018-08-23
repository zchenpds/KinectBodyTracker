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

## Cloning
Since it contains a submodule, please use option "--recurse-submodule" to clone to a local directory.
```
git clone --recurse-submodule https://github.com/zchenpds/KinectBodyTracker
```

## Compiling the source
1. Before compiling the source, you might need to retarget the solution if you have a higher version of Windows SDK. To do this, right click on your solution in the Solution Explorer, and then choose "Retarget solution". 
1. Once the source code is compiled, a `.exe` file will be generated in `.\Release` in case of Release mode, or in `.\Debug` in case of Debug mode. Currently, only the Release build works.


## Output File Format
1.	tO	:	Last available Odroid timestamp (uSecs)
1.	tOW	:	Windows timestamp associated with the last available Odroid timestamp (mSecs)
1.	tK	:	Last available Kinect timestamp (mSecs)
1.	tKW	:	Windows timestamp associated with the last available Kinect timestamp (mSecs)
1.	kneeLX	:	Left knee x coordinate (meters, relative to Camera frame)
1.	kneeLY	:	Left knee y coordinate (meters, relative to Camera frame)
1.	kneeLZ	:	Left knee z coordinate (meters, relative to Camera frame)
1.	ankleLX	:	Left ankle x coordinate (meters, relative to Camera frame)
1.	ankleLY	:	Left ankle y coordinate (meters, relative to Camera frame)
1.	ankleLZ	:	Left ankle z coordinate (meters, relative to Camera frame)
1.	footLX	:	Left foot x coordinate (meters, relative to Camera frame)
1.	footLY	:	Left foot y coordinate (meters, relative to Camera frame)
1.	footLZ	:	Left foot z coordinate (meters, relative to Camera frame)
1.	kneeRX	:	Right knee x coordinate (meters, relative to Camera frame)
1.	kneeRY	:	Right knee y coordinate (meters, relative to Camera frame)
1.	kneeRZ	:	Right knee z coordinate (meters, relative to Camera frame)
1.	ankleRX	:	Right ankle x coordinate (meters, relative to Camera frame)
1.	ankleRY	:	Right ankle y coordinate (meters, relative to Camera frame)
1.	ankleRZ	:	Right ankle z coordinate (meters, relative to Camera frame)
1.	footRX	:	Right foot x coordinate (meters, relative to Camera frame)
1.	footRY	:	Right foot y coordinate (meters, relative to Camera frame)
1.	footRZ	:	Right foot z coordinate (meters, relative to Camera frame)
1.	tRW	:	Windows timestamp asscociated with the last Robot state update.
1.	x	:	Robot x coordinate (meters, relative to initial pose, aka World frame)
1.	y	:	Robot y coordinate (meters, relative to initial pose, aka World frame)
1.	th	:	Robot orientation (radians, relative to initial pose, aka World frame)
1.	v	:	Robot linear velocity (meters/sec)
1.	w	:	Robot angular velocity (radians/sec)
1.	batVolt	:	Battery voltage (volts)
1.	isFollowing	:	Indicating whether following is enabled (0/1)
1.	xVm	:	x coordinate of the virtual marker's current position (meters, relative to the world frame)
1.	yVm	:	y coordinate of the virtual marker's current position (meters, relative to the world frame)
1.	xVmG	:	x coordinate of the virtual marker's goal position (meters, relative to the world frame)
1.	yVmG	:	y coordinate of the virtual marker's goal position (meters, relative to the world frame)
1.	vD	:	Desired linear velocity (meters/sec)
1.	wD	:	Desired angular velocity (radians/sec)
1.	dVmG	:	Desired distance of subject
1.	hVmG	:	Desired heading of subject
1.	vScale	:	Proportional coefficient for control of linear velocity (dimensionless)
1.	wScale	:	Proportional coefficient for control of angular velocity (dimensionless)
