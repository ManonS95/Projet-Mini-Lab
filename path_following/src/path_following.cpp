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
#include "planning/RRTPlanning.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commande");
    ros::NodeHandle n;

    Commande cmd;
    Pose_2d p;
	geometry_msgs::Transform start, goal;
	start.translation.x = 1200.0;
	start.translation.y = 1000.0;
	goal.translation.x = 800.0;
	goal.translation.y = 800.0;
	
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");
	nav_msgs::GetMap srv;
    nav_msgs::OccupancyGrid original_map;

    // Récupèrer la Map
    client.waitForExistence();
    if (client.call(srv))
    {
        original_map =  srv.response.map;
		ros::ServiceClient client_plan = n.serviceClient<planning::RRTPlanning>("plan_srv");
		planning::RRTPlanning srv_plan;
		srv_plan.request.start = start;
		srv_plan.request.goal = goal;
		srv_plan.request.map = original_map;
    
		client_plan.waitForExistence();
		if (client_plan.call(srv_plan))
		{
			cmd.init(srv_plan.response.path);
		}
		else
		{
			ROS_ERROR("Failed to call service plan_srv");
			return 1;
		}
	}
	else
    {
        ROS_ERROR("Failed to call service get_map");
        return 1;
    }
	
    ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Rate rate_path(10);

	while(!cmd.fin() && ros::ok())
	{
		cout<<"Coucou1"<<endl;
		ros::Subscriber sub_tf = n.subscribe("tf", 1, &Pose_2d::init, &p);
		vector<double> v = cmd.command_law(p.getX(), p.getY(), p.getTheta());
		cout<<"Coucou3"<<endl;
		geometry_msgs::Twist t;
		t.linear.x = v.at(0);
		t.angular.z = v.at(1);
		cout<<"Coucou4"<<endl;
		pub_cmd.publish(t);
		rate_path.sleep();
	}
	geometry_msgs::Twist t;
	pub_cmd.publish(t);


    ros::spin();

    return 0;
}
