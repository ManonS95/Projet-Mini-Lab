#ifndef POSE_2D_HPP
#define POSE_2D_HPP

#include <geometry_msgs/Pose.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>

class Pose_2d
{
	private:
		double x;
		double y;
		double theta;
		geometry_msgs::Transform pose;

	public:
		Pose_2d();
		void init(const tf2_msgs::TFMessage& pose);
		double getX();
		double getY();
		double getTheta();
};

#endif //POSE_2D_HPP
