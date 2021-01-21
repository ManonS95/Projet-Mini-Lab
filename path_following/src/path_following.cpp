#include "ros/ros.h"
#include "commande.hpp"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <time.h>
#include <stdio.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commande");
    ros::NodeHandle n;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    Commande cmd;
    Pose_2D p;
    ros::Subscriber sub = n.subscribe("rrt_path", 1, cmd.init);
	while(!cmd.fin())
	{
		ros::Subscriber sub = n.subscribe("tf2", 1, p.init);
		vector<double> v = cmd.commande_law(p.x(), p.y(), p.theta())
	}


    
    ros::spin();

    return 0;
}
