#include "ros/ros.h"
#include "commande.hpp"
#include "pose_2d.hpp"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <time.h>
#include <stdio.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commande");
    ros::NodeHandle n;

    Commande cmd;
    Pose_2d p;
    ros::Subscriber sub_path = n.subscribe("rrt_path", 1, &Commande::init, &cmd);
    ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	while(!cmd.fin())
	{
		ros::Subscriber sub_tf = n.subscribe("tf", 1, &Pose_2d::init, &p);
		vector<double> v = cmd.command_law(p.getX(), p.getY(), p.getTheta());
		geometry_msgs::Twist t;
		t.linear.x = v.at(0);
		t.angular.z = v.at(1); 
		pub_cmd.publish(t);
	}
	geometry_msgs::Twist t;
	pub_cmd.publish(t);


    
    ros::spin();

    return 0;
}
