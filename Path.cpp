//#include "pch.h"
#include "Path.h"
#include <tf\tf.h>
#include <math.h>

namespace BodyTracker {

	Path::Path()
	{
		half_dist_circle = 1.4;
		radius = 0.8;
		// Check validity of the parameters;
		if (radius > half_dist_circle) {
			// Parameter radius cannot exceed half_dist_circle! Setting radius to half_dist_circle.
			radius = half_dist_circle;
		}
		alpha = asin(radius / half_dist_circle);
		straight_part_len = radius / tan(alpha);
		turn_angle = M_PI + 2 * alpha;
		curved_part_len = radius * turn_angle;
		circumference = straight_part_len * 4 + curved_part_len * 2;

		setPathPose(0, 0, 0);

	}


	Path::~Path()
	{
	}

	void Path::setPathPose(double x, double y, double th)
	{
		m_PathPose.position.x = x;
		m_PathPose.position.y = y;
		m_PathPose.position.z = 0;
		geometry_msgs::Quaternion quatDesired = tf::createQuaternionFromYaw(th);

		m_PathPose.orientation.x = quatDesired.x;
		m_PathPose.orientation.y = quatDesired.y;
		m_PathPose.orientation.z = quatDesired.z;
		m_PathPose.orientation.w = quatDesired.w;
	}

	geometry_msgs::Pose * Path::getPoseOnPath(double dist)
	{
		double x, y, th, x0, y0, th0;
		x0 = m_PathPose.position.x;
		y0 = m_PathPose.position.y;
		th0 = asin(m_PathPose.orientation.z) * 2;

		dist -= floor(dist / circumference) * circumference;

		if (dist < straight_part_len) {
			x = x0 + dist * cos(th0);
			y = y0 + dist * sin(th0);
			th = th0;
		}
		else if (dist < straight_part_len + curved_part_len) {
			double x1 = x0 + radius / sin(alpha) * cos(th0 - alpha);
			double y1 = y0 + radius / sin(alpha) * sin(th0 - alpha);
			double angle_turned = (dist - straight_part_len) / radius;
			double turn_start_angle = M_PI / 2 + th0;
			x = x1 + radius * cos(turn_start_angle - angle_turned);
			y = y1 + radius * sin(turn_start_angle - angle_turned);
			th = th0 - angle_turned;
		}
		else if (dist < straight_part_len * 3 + curved_part_len) {
			double seg_dist = dist - straight_part_len - curved_part_len;
			x = x0 + (straight_part_len - seg_dist) * cos(th0 - alpha * 2);
			y = y0 + (straight_part_len - seg_dist) * sin(th0 - alpha * 2);
			th = th0 - turn_angle;
		}
		else if (dist < straight_part_len * 3 + curved_part_len * 2) {
			double x1 = x0 - radius / sin(alpha) * cos(th0 - alpha);
			double y1 = y0 - radius / sin(alpha) * sin(th0 - alpha);
			double angle_turned = (dist - straight_part_len * 3 - curved_part_len) / radius;
			double turn_start_angle = M_PI / 2 + th0 - 2 * alpha;
			x = x1 + radius * cos(turn_start_angle + angle_turned);
			y = y1 + radius * sin(turn_start_angle + angle_turned);
			th = th0 - turn_angle + angle_turned;
		}
		else {
			double seg_dist = dist - circumference;
			x = x0 + seg_dist * cos(th0);
			y = y0 + seg_dist * sin(th0);
			th = th0;
		}

		m_PoseDesired.position.x = x;
		m_PoseDesired.position.y = y;

		while (th < -M_PI) th += 2 * M_PI;
		while (th > M_PI) th -= 2 * M_PI;
		geometry_msgs::Quaternion quatDesired = tf::createQuaternionFromYaw(th);

		m_PoseDesired.orientation.x = quatDesired.x;
		m_PoseDesired.orientation.y = quatDesired.y;
		m_PoseDesired.orientation.z = quatDesired.z;
		m_PoseDesired.orientation.w = quatDesired.w;

		return &m_PoseDesired;
	}

	double Path::getCircumference()
	{
		return circumference;
	}

};