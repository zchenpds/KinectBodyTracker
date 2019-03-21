#pragma once

#include <geometry_msgs\Pose.h>


#ifndef M_PI
const double M_PI = 3.1415927;
#endif // of M_PI, windows has a function call instead of a define

namespace BodyTracker {
	class Path
	{
	private:
		geometry_msgs::Pose m_PoseDesired;
		geometry_msgs::Pose m_PathPose;

		double circumference;
		double half_dist_circle;
		double radius;
	
		double alpha;
		double straight_part_len;
		double turn_angle;
		double curved_part_len;

	public:
		Path();
		~Path();
		void setPathPose(double x, double y, double th);
		geometry_msgs::Pose *getPoseOnPath(double dist);
		double getCircumference();
	};

};