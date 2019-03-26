#pragma once

#include <geometry_msgs\Pose.h>
#include "Config.h"

#ifndef M_PI
const double M_PI = 3.1415927;
#endif // of M_PI, windows has a function call instead of a define

namespace BodyTracker {

	class BasePath {
	protected:
		geometry_msgs::Pose m_PoseDesired;
		geometry_msgs::Pose m_PathPose;
		double circumference;
	public:
		BasePath();
		void setPathPose(double x, double y, double th);
		void setPathPose(geometry_msgs::Pose* pPathPose);
		virtual geometry_msgs::Pose *getPoseOnPath(double dist) = 0;
		double getCircumference() const;
		virtual void setParams() = 0;
		virtual std::string getType() const = 0;
	};

	class PathEight : public BasePath
	{
	private:
		double half_dist_circle;
		double radius;
	
		double alpha;
		double straight_part_len;
		double turn_angle;
		double curved_part_len;

	public:
		PathEight();
		~PathEight();
		geometry_msgs::Pose *getPoseOnPath(double dist) override;
		void setParams() override;
		std::string getType() const override;
	};

};