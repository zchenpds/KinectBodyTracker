# KinectBodyTracker

## Description
This Visual Studio project is adapted from an example of Kinect SDK 2.0 named BodyBasics. It features:
1. Extracting the position of the spine base of the subject;
1. Calculating the linear and angular velocity commands for a differential drive robot to follow the subject or to move the robot along a predefined path;
1. Transforming the coordinates of the body joint positions from a Kinect frame to a inertial frame;
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


## Format of the logged files
### data_yyyymmdd_hh_mm_ss_Kinect0.csv

-.	**tO**	:	Last available Odroid timestamp (uSecs)
-.	**tOW**	:	Windows timestamp associated with the last available Odroid timestamp (mSecs)
-.	**trigger**:	sync signal
-.	**tK**	:	Last available Kinect timestamp (mSecs)
-.	**tKW**	:	Windows timestamp associated with the last available Kinect timestamp (mSecs)
-.	**KLKneeX, KLKneeY, KLKneeZ, **
	**KLAnkleX, KLAnkleY, KLAnkleZ, **
	**KLFootX, KLFootY, KLFootZ, **
	**KRKneeX, KRKneeY, KRKneeZ, **
	**KRAnkleX, KRAnkleY, KRAnkleZ,**
	**KRFootX, KRFootY, KRFootZ**:	
	Left/Right knee/ankle/foot x/y/z coordinates (meters, relative to Camera frame)
-.	**tDiffR**	:	the time difference between the previous robot data packet and the Kinect data packet. (The robot position corresponding to this packet of Kinect data must be extrapolated/inferred from the previous robot data packet.)
-.	**WLKneeX, WLKneeY, WLKneeZ, **
	**WLAnkleX, WLAnkleY, WLAnkleZ,**
	**WLFootX, WLFootY, WLFootZ,**
	**WRKneeX, WRKneeY, WRKneeZ,**
	**WRAnkleX, WRAnkleY, WRAnkleZ,**
	**WRFootX, WRFootY, WRFootZ**:
	Left/Right knee/ankle/foot x/y/z coordinates (meters, relative to World frame)
	
### data_yyyymmdd_hh_mm_ss_Robot0.csv
-.	**tR**	:	Windows timestamp at which the last SIP packet was received.
-.	**tRW**	:	Windows timestamp asscociated with the last Robot state update.
-.	**x**	:	Robot x coordinate (meters, relative to initial pose, aka World frame)
-.	**y**	:	Robot y coordinate (meters, relative to initial pose, aka World frame)
-.	**th**	:	Robot orientation (radians, relative to initial pose, aka World frame)
-.	**v**	:	Robot linear velocity (meters/sec)
-.	**w**	:	Robot angular velocity (radians/sec)
-.	**batVolt**	:	Battery voltage (volts)
-.	**isFollowing**	:	Indicating whether following is enabled (0/1)
-.	**xVm**	:	x coordinate of the virtual marker's current position (meters, relative to the world frame)
-.	**yVm**	:	y coordinate of the virtual marker's current position (meters, relative to the world frame)
-.	**dist**	:	the arc length the robot has traveled along the predefined path.
-.	**xVmG**	:	x coordinate of the virtual marker's goal position (meters, relative to the world frame)
-.	**yVmG**	:	y coordinate of the virtual marker's goal position (meters, relative to the world frame)
-.	**vD**	:	Desired linear velocity (meters/sec)
-.	**wD**	:	Desired angular velocity (radians/sec)
-.	**distVm**	:	Desired distance of virtual marker
-.	**headVm**	:	Desired heading of virtual marker
-.	**vScale**	:	Proportional coefficient for control of linear velocity (dimensionless)
-.	**wScale**	:	Proportional coefficient for control of angular velocity (dimensionless)
-.	**distDesired**	:	Desired distance of the subject from the robot
-.	**thDesired**	:	Desired heading
-.	**xError, yError**	:	x and y coordinates of the path following controller.
-.	**vx, vy**	: x and y coordinates of the desired speed.
-.	**vNew, wNew**	: The linear and angular velocity commands calculated from the path following error.
-.	**xVmD, yVmD**	: (Forget about this)
