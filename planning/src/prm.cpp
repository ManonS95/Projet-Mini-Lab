#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <time.h>


int** random_init(int nb_points, int width, int height)
{
	srand (time (NULL));
	int** randomValue = (int**)malloc(nb_points * sizeof(int*));
	
	for (int i = 0; i < nb_points; i++)
	{
		randomValue[i] = (int*)malloc(2 * sizeof(int));
		randomValue[i][0] = rand() % height;
		randomValue[i][1] = rand() % width;
	}
	return randomValue;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_map");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
  nav_msgs::GetMap srv;
  nav_msgs::OccupancyGrid map;
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
  int taille = 0.7 * map.info.width * map.info.height;
  int** tab = random_init(taille, map.info.width, map.info.height);
  for (int i = 0; i < taille; i++)
  {
  	ROS_INFO("Tab[%d] = [%d, %d]\n", i, tab[i][0], tab[i][1]);
  }
  
  ros::spin();

  return 0;
}
