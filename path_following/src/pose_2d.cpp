#include "pose_2d.hpp"
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

Pose_2d::Pose_2d(): x(0), y(0), theta(0)
{}

void Pose_2d::init(const geometry_msgs::TransformStamped& pose)
{
	this->pose = pose.transform;
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
	//cout << "theta : " << theta << endl;
	return theta;
}
