#include "ros/ros.h"
#include "commande.hpp"
#include "pose_2d.hpp"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <time.h>
#include <stdio.h>
#include "planning/RRTPlanning.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


using namespace std;

Pose_2d p;

void robCallback(const nav_msgs::Odometry& msg)
{
	p.init(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commande");
    ros::NodeHandle n;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

    Commande cmd;
	geometry_msgs::Transform start, goal;
	geometry_msgs::TransformStamped transformStamped;
	char map_type;
	ros::ServiceClient client;

	cout << "Quelle map voulez-vous utiliser ? (static -> 1, dynamic -> 2)" << endl;
	cin >> map_type;
	
	if (map_type == '1')
	{
		client = n.serviceClient<nav_msgs::GetMap>("static_map");
		ros::Subscriber sub_rob = n.subscribe("odom", 100, robCallback);
	}
	else
	{
		try
		{
			transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(100.0));
			p.init(transformStamped);
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
	}

	start.translation.x = p.getX();//1200.0;
	start.translation.y = p.getY();//1000.0;
	goal.translation.x = 0.12;
	goal.translation.y = -7.74;

	nav_msgs::GetMap srv;
    nav_msgs::OccupancyGrid original_map;

    // Récupèrer la Map
    client.waitForExistence();
    if (client.call(srv))
    {
        original_map = srv.response.map;
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
	geometry_msgs::Twist t;


	while(!cmd.fin() && n.ok())
	{
		if (map_type == '2')
		{
			try
			{
				transformStamped = tfBuffer.lookupTransform("map", "base_footprint", ros::Time(0));
				p.init(transformStamped);
			}
			catch (tf2::TransformException &ex)
			{
				ROS_WARN("%s", ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
		}
		
		vector<double> v = cmd.command_law(p.getX(), p.getY(), p.getTheta());

		t.linear.x = v.at(0);
		t.angular.z = v.at(1);
		pub_cmd.publish(t);
		rate_path.sleep();
	}
	t.linear.x = 0;
	t.angular.z = 0;
	pub_cmd.publish(t);


    ros::spin();

    return 0;
}
