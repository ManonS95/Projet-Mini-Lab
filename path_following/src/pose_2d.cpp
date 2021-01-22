#include "pose_2d.hpp"
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

Pose_2d::Pose_2d(): x(0), y(0), theta(0)
{}

void Pose_2d::init(const tf2_msgs::TFMessage& pose)
{
	for (const auto& p : pose.transforms)
	{
		if (p.child_frame_id.compare("base_footprint") == 0)
		{
			this->pose = p.transform;
		}
	}
}

double Pose_2d::getX()
{
	x = pose.translation.x;
	return x;
}

double Pose_2d::getY()
{
	y = pose.translation.y;
	return y;
}

double Pose_2d::getTheta()
{
	tf2::Quaternion q(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
	tf2::Matrix3x3 m(q);
	tf2Scalar a, b;
	m.getRPY(a, b, theta);
	return theta;
}
