#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <stdio.h>
#include <fcntl.h>

geometry_msgs::Twist action(char ch)
{
	geometry_msgs::Twist twist;
	if (ch == 'z')
	{
		twist.linear.x = 1, twist.linear.y = 0, twist.linear.z = 0;
		twist.angular.x = 0, twist.angular.y = 0, twist.angular.z = 0;
	}
	else if (ch == 'q')
	{
		twist.linear.x = 0, twist.linear.y = 0, twist.linear.z = 0;
		twist.angular.x = 0, twist.angular.y = 0, twist.angular.z = 1;
	}
	else if (ch == 'x')
	{
		twist.linear.x = -1, twist.linear.y = 0, twist.linear.z = 0;
		twist.angular.x = 0, twist.angular.y = 0, twist.angular.z = 0;
	}
	else if (ch == 'd')
	{
		twist.linear.x = 0, twist.linear.y = 0, twist.linear.z = 0;
		twist.angular.x = 0, twist.angular.y = 0, twist.angular.z = -1;
	}
	else
	{
		twist.linear.x = 0, twist.linear.y = 0, twist.linear.z = 0;
		twist.angular.x = 0, twist.angular.y = 0, twist.angular.z = 0;
	}
	
	return twist;
}


int kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}
    


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "teleop_minilab");
  	ros::NodeHandle nh_;
  	ros::Publisher vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	char choice;
	geometry_msgs::Twist twist;
	
	while(1)
	{
        if( kbhit() )
        {
            choice = getchar();
			twist = action(choice);
			vel_pub_.publish(twist);
		}
	}
	ros::spin();
	
	return 0;
}


