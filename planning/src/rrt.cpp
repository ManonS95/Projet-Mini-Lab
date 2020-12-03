#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <time.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_map");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
  nav_msgs::GetMap srv;
  nav_msgs::OccupancyGrid map;

  // Récupèrer la Map
  if (client.call(srv))
  {
    map =  srv.response.map;
    ROS_INFO("We have the map!\n");
  }
  else
  {
    ROS_ERROR("Failed to call service get_map");
    return 1;
  }

  // Récupérer position start et goal
  // Construction tree



  
  ros::spin();

  return 0;
}
