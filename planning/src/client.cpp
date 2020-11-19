#include "ros/ros.h"
#include <nav_msgs/GetMap.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_map");
  

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
  nav_msgs::GetMap srv;
  if (client.call(srv))
  {
  	ROS_INFO("We have the map!\n");
  }
  else
  {
    ROS_ERROR("Failed to call service get_map");
    return 1;
  }
  ros::spin();

  return 0;
}
