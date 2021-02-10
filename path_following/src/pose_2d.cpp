#include "pose_2d.hpp"
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;

Pose_2d::Pose_2d(): x(0), y(0), theta(0), l1(1)
{}

void Pose_2d::init(const geometry_msgs::TransformStamped& pose)
{
	x = pose.transform.translation.x;
	y = pose.transform.translation.y;
	quat = pose.transform.rotation;
}

void Pose_2d::init(const nav_msgs::Odometry& pose)
{
	this->x = pose.pose.pose.position.x;
	this->x = pose.pose.pose.position.x;
	this->quat = pose.pose.pose.orientation;
}

double Pose_2d::getX()
{
	theta = getTheta();
	return x + l1*cos(theta);
}

double Pose_2d::getY()
{
	theta = getTheta();
	return y  + l1*sin(theta);
}

double Pose_2d::getTheta()
{
	tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf2::Matrix3x3 m(q);
	tf2Scalar a, b;
	m.getRPY(a, b, theta);
	//cout << "theta : " << theta << endl;
	return theta;
}
