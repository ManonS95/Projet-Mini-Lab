#ifndef POSE_2D_HPP
#define POSE_2D_HPP

#include <geometry_msgs/Pose.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>


class Pose_2d
{
	private:
		double x;
		double y;
		double theta;
		double l1;
		geometry_msgs::Quaternion quat;
		geometry_msgs::Transform pose;

	public:
		Pose_2d();
		void init(const geometry_msgs::TransformStamped& pose);
		void init(const nav_msgs::Odometry& pose);
		double getX();
		double getY();
		double getTheta();
};

#endif //POSE_2D_HPP
