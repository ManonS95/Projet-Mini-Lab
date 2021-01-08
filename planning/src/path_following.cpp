#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <time.h>
#include <stdio.h>


using namespace std;



void chatterCallback(const nav_msgs::Path& msg)
{
    for (size_t i = 0; i < msg.poses.size(); i++)
    {
        printf("Position %d = (%f, %f)", i, msg.poses.at(i).pose.position.x, msg.poses.at(i).pose.position.y);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_path");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("rrt_path", 1000, chatterCallback);
    ros::spin();

    return 0;
}
