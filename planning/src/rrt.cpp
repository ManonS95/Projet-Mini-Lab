#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <time.h>
#include "Tree.hpp"
#include "Vertex.hpp"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_map");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap srv;
    nav_msgs::OccupancyGrid map;
    cv::Mat image;
    cv::Mat outImage;

    // Récupèrer la Map
    if (client.call(srv))
    {
        map =  srv.response.map;
        ROS_INFO("We have the map!\n");
        image = cv::Mat(map.info.height, map.info.width, CV_8UC3, cv::Scalar::all(0));
        
        for (size_t x = 0; x < map.info.height; x++)
        {
            for (size_t y = 0; y < map.info.width; y++)
            {
                if (map.data[y + map.info.width * x] == -1)
                {
                    image.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 255, 255);
                }
                else if (map.data[y + map.info.width * x] == 0)
                {
                    image.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 255, 255);
                }
                else if (map.data[y + map.info.width * x] > 0)
                {
                    image.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 0);
                }
            }
        }
    }
    else
    {
        ROS_ERROR("Failed to call service get_map");
        return 1;
    }


    // Récupérer position start et goal
    Vertex start(800, 800);
    Vertex goal(900, 800);
    cv::circle(image, cv::Point(start.getPosPix()[0], start.getPosPix()[1]), 20, cv::Scalar(255, 0, 0), -1);
    cv::circle(image, cv::Point(goal.getPosPix()[0], goal.getPosPix()[1]), 20, cv::Scalar(0, 255, 0), -1);



    // Construction tree
    Tree t = build_rrt(start, goal, map);
    vector<Vertex> path = t.getPath(goal);
    vector<Vertex> tree = t.getTree();

    for(size_t i = 0; i < tree.size(); i++)
    {
        cv::circle(image, cv::Point(tree.at(i).getPosPix()[0], tree.at(i).getPosPix()[1]), 10, cv::Scalar(255, 255, 0), -1);
    }

    cv::resize(image, outImage, cv::Size(image.cols * 0.7, image.rows * 0.7), 0, 0, CV_INTER_LINEAR);
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    //outImage = cv::Mat(outImage, cv::Rect(10, 10, 90, 90)); // using a rectangle
    imshow("Display Image", outImage);
    cv::waitKey(0);


    ros::spin();

    return 0;
}
