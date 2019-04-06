//#include "pch.h"
#include "Path.h"
#include <tf\tf.h>
#include <math.h>
#include "Config.h"

namespace BodyTracker {

	// *** BasePath implementation ***

	BasePath::BasePath(): circumference(0)
	{
		setPathPose(0, 0, 0);
	}

	void BasePath::setPathPose(double x, double y, double th)
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

	void BasePath::setPathPose(geometry_msgs::Pose * pPathPose)
	{
		setPathPose(pPathPose->position.x, pPathPose->position.y, pPathPose->orientation.z);
	}

	double BasePath::getCircumference() const
	{
		if (circumference > 0)
			return circumference;
		else
			; throw std::runtime_error("BasePath::getCircumference():\n circumference not set. \n\n");
	}

	// *** PathEight implementation ***

	PathEight::PathEight() : BasePath()
	{
		setParams();
	}


	PathEight::~PathEight()
	{
	}

	

	geometry_msgs::Pose * PathEight::getPoseOnPath(double dist, float * pMaxV)
	{
		double x, y, th, x0, y0, th0;
		x0 = m_PathPose.position.x;
		y0 = m_PathPose.position.y;
		th0 = asin(m_PathPose.orientation.z) * 2;

		dist -= floor(dist / getCircumference()) * getCircumference();
		
		// Speed constraint
		float maxV = 0;
		float vcc = sqrt(aLateralMax * radius);
		
		// Segment endpoints definition
		double distEndpoints[] = { 
			straight_part_len,
			straight_part_len + curved_part_len,
			straight_part_len * 3 + curved_part_len,
			straight_part_len * 3 + curved_part_len * 2
		};

		// Which segment does dist correspond to?
		if (dist < distEndpoints[0]) {
			x = x0 + dist * cos(th0);
			y = y0 + dist * sin(th0);
			th = th0;
			maxV = sqrt(2 * aLongitudinalMax * (distEndpoints[0] - dist) + vcc * vcc);
		}
		else if (dist < distEndpoints[1]) {
			double x1 = x0 + radius / sin(alpha) * cos(th0 - alpha);
			double y1 = y0 + radius / sin(alpha) * sin(th0 - alpha);
			double angle_turned = (dist - straight_part_len) / radius;
			double turn_start_angle = M_PI / 2 + th0;
			x = x1 + radius * cos(turn_start_angle - angle_turned);
			y = y1 + radius * sin(turn_start_angle - angle_turned);
			th = th0 - angle_turned;
			maxV = vcc;
		}
		else if (dist < distEndpoints[2]) {
			double seg_dist = dist - distEndpoints[1];
			x = x0 + (straight_part_len - seg_dist) * cos(th0 - alpha * 2);
			y = y0 + (straight_part_len - seg_dist) * sin(th0 - alpha * 2);
			th = th0 - turn_angle;
			maxV = sqrt(2 * aLongitudinalMax * (distEndpoints[2] - dist) + vcc * vcc);
		}
		else if (dist < distEndpoints[3]) {
			double x1 = x0 - radius / sin(alpha) * cos(th0 - alpha);
			double y1 = y0 - radius / sin(alpha) * sin(th0 - alpha);
			double angle_turned = (dist - distEndpoints[2]) / radius;
			double turn_start_angle = M_PI / 2 + th0 - 2 * alpha;
			x = x1 + radius * cos(turn_start_angle + angle_turned);
			y = y1 + radius * sin(turn_start_angle + angle_turned);
			th = th0 - turn_angle + angle_turned;
			maxV = vcc;
		}
		else {
			double seg_dist = dist - getCircumference();
			x = x0 + seg_dist * cos(th0);
			y = y0 + seg_dist * sin(th0);
			th = th0;
			maxV = sqrt(2 * aLongitudinalMax * (getCircumference() + distEndpoints[0] - dist) + vcc * vcc);
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

		if (pMaxV) {
			*pMaxV = maxV;
		}

		return &m_PoseDesired;
	}

	void PathEight::setParams()
	{
		Config* pConfig = Config::Instance();
		half_dist_circle = 1.4;
		pConfig->assign("PathEight/half_dist_circle", half_dist_circle);
		radius = 0.8;
		pConfig->assign("PathEight/radius", radius);

		aLongitudinalMax = 0.3;
		aLateralMax = 0.2;
		pConfig->assign("aLateralMax", aLateralMax);

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
	}

	std::string PathEight::getType() const
	{
		return std::string("PathEight");
	}


	

	PathU::PathU() : BasePath()
	{
		setParams();
	}

	geometry_msgs::Pose * PathU::getPoseOnPath(double dist, float * pMaxV)
	{
		float x, y, th, x0, y0, th0;
		x0 = m_PathPose.position.x;
		y0 = m_PathPose.position.y;
		th0 = asin(m_PathPose.orientation.z) * 2;
		const Eigen::Transform<float, 2, Eigen::Affine> transform = Eigen::Rotation2Df(th0) * Eigen::Translation2f(x0, y0);

		dist -= floor(dist / getCircumference()) * getCircumference();

		// Speed constraint
		float maxV = 0;
		float vcc = sqrt(aLateralMax * radius);

		// Segment endpoints definition
		double distEndpoints[] = {
			straight_part_len,
			straight_part_len + curved_part_len,
			straight_part_len * 2 + curved_part_len
		};

		// Which segment does dist correspond to?
		if (dist < distEndpoints[0]) {
			x = dist;
			y = 0;
			th = 0;
			maxV = sqrt(2 * aLongitudinalMax * (distEndpoints[0] - dist) + vcc * vcc);
		}
		else if (dist < distEndpoints[1]) {
			float x1 = straight_part_len;
			float y1 = -radius;
			float angle_turned = (dist - distEndpoints[0]) / radius;
			float turn_start_angle = M_PI / 2;
			x = x1 + radius * cos(turn_start_angle - angle_turned);
			y = y1 + radius * sin(turn_start_angle - angle_turned);
			th = - angle_turned;
			maxV = vcc;
		}
		else if (dist < distEndpoints[2]) {
			float seg_dist = dist - distEndpoints[1];

			x = straight_part_len - seg_dist;
			y = - 2 * radius;
			th = -M_PI;
			maxV = sqrt(2 * aLongitudinalMax * (distEndpoints[2] - dist) + vcc * vcc);
		}
		else {
			float x1 = 0;
			float y1 = -radius;
			float angle_turned = (dist - distEndpoints[2]) / radius;
			float turn_start_angle = -M_PI / 2;
			x = x1 + radius * cos(turn_start_angle - angle_turned);
			y = y1 + radius * sin(turn_start_angle - angle_turned);
			th = -M_PI - angle_turned;
			maxV = vcc;
		}

		Eigen::Vector2f p = transform * Eigen::Vector2f(x, y);

		m_PoseDesired.position.x = p(0);
		m_PoseDesired.position.y = p(1);

		th += th0;
		while (th < -M_PI) th += 2 * M_PI;
		while (th > M_PI) th -= 2 * M_PI;
		geometry_msgs::Quaternion quatDesired = tf::createQuaternionFromYaw(th);

		m_PoseDesired.orientation.x = quatDesired.x;
		m_PoseDesired.orientation.y = quatDesired.y;
		m_PoseDesired.orientation.z = quatDesired.z;
		m_PoseDesired.orientation.w = quatDesired.w;

		if (pMaxV) {
			*pMaxV = maxV;
		}

		return &m_PoseDesired;
	}

	void PathU::setParams()
	{
		Config* pConfig = Config::Instance();
		straight_part_len = 1.5;
		pConfig->assign("PathU/straight_part_len", straight_part_len);
		radius = 1.5;
		pConfig->assign("PathU/radius", radius);

		aLongitudinalMax = 0.3;
		aLateralMax = 0.2;
		pConfig->assign("aLateralMax", aLateralMax);

		curved_part_len = radius * M_PI;
		circumference = straight_part_len * 2 + curved_part_len * 2;
	}

	std::string PathU::getType() const
	{
		return std::string("PathU");
	}

};