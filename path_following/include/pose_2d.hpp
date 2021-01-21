#ifndef POSE_2D_HPP
#define POSE_2D_HPP

#include <geometry_msgs/Pose.h>

class Pose_2d
{
	private:
		double x;
		double y;
		double theta;
		Pose pose;

	public:
		Pose_2d();
		init(Pose pose);
		x();
		y();
		theta();
};

#endif //POSE_2D_HPP
