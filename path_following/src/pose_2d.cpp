#include "pose_2d.hpp"

using namespace std;

Pose_2d::Pose_2d(): x(0), y(0), theta(0)
{
}

Pose_2d::init(Pose pose)
{
	this->pose = pose;		
}

Pose_2d::x()
{
	x = pose.position.x;
	return x;
}

Pose_2d::y()
{
	y = pose.position.y;
	return y;
}

Pose_2d::theta()
{
	Matrix3x3 m(pose.orientation);
	m.getRPY();
	return theta;
}
