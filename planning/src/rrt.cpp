#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <time.h>
#include "Tree.hpp"
#include "Vertex.hpp"
#include "Dijkstra.hpp"
#include "planning/RRTPlanning.h"

using namespace std;

nav_msgs::OccupancyGrid map_dilatation(const nav_msgs::OccupancyGrid &map, int r_robot)
{
    nav_msgs::OccupancyGrid new_map = map;
    int cpt_x;

    
    for (size_t y = 1; y < map.info.height; y++)
    {
        for (size_t x = 1; x < map.info.width; x++)
        {
            if (map.data[x + map.info.width * y] != 0) // Si le pixel est noir
            {
                /*if ((map.data[x - 1 + map.info.width * y] == 0) || (map.data[x + 1 + map.info.width * y] == 0) && (map.data[x + map.info.width * (y - 1)] == 0) && (map.data[x + map.info.width * (y + 1)] == 0))
                {*/
                    for (int i = 0; i < 2 * r_robot; i++)
                    {
                        if (x + i - r_robot < 0)
                        {
                            new_map.data[i + new_map.info.width * y] = 1;
                            cpt_x = i;
                        }
                        else if (x + i - r_robot < map.info.width)
                        {
                            new_map.data[x + i - r_robot + new_map.info.width * y] = 1;
                            cpt_x = x + i - r_robot;
                        }
                        for (int j = 0; j < 2 * r_robot; j++)
                        {
                            if (y + j - r_robot < 0)
                            {
                                new_map.data[cpt_x + new_map.info.width * j] = 1;
                            }
                            else if (y + j - r_robot < map.info.height)
                            {
                                new_map.data[cpt_x + new_map.info.width * (y + j - r_robot)] = 1;
                            }
                        }
                    }
                //}
            }
            cout << "x = " << x << " y = " << y << endl;
        }
    }

    return new_map;
}

bool planning_function(planning::RRTPlanning::Request& req, planning::RRTPlanning::Response& res)
{
    nav_msgs::OccupancyGrid map;
    cv::Mat image;
    cv::Mat outImage;

    
    map = map_dilatation(req.map, 10);
    cout << "repère = " << map.header.frame_id << endl;
    ROS_INFO("We have the map!\n");
    image = cv::Mat(map.info.height, map.info.width, CV_8UC3, cv::Scalar::all(0));
    
    for (size_t y = 1; y < map.info.height; y++)
    {
        for (size_t x = 1; x < map.info.width; x++)
        {
            if (map.data[x + map.info.width * y] == 0)
            {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // Inside map -> White
            }
            else if (map.data[x + map.info.width * y] != 0)
            {
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Wall -> Black
            }
        }
    }


    // Récupérer position start et goal
    int start_x = round((req.start.translation.x - map.info.origin.position.x) / map.info.resolution);
    int start_y = round((req.start.translation.y - map.info.origin.position.y) / map.info.resolution);

    Vertex start(start_x, 1000);//start_y);
    Vertex goal(req.goal.translation.x, req.goal.translation.y);

    cv::circle(image, cv::Point(start.getPosPix()[0], start.getPosPix()[1]), 10, cv::Scalar(255, 0, 0), -1);
    cv::circle(image, cv::Point(goal.getPosPix()[0], goal.getPosPix()[1]), 10, cv::Scalar(0, 255, 0), -1);



    // Construction tree
    /*Tree t = build_rrt(start, goal, map);
    vector<Vertex> path = t.getPath(goal);
    vector<Vertex> tree = t.getTree();*/

    vector<Vertex> path = rrt_connect_planner(start, goal, map);

	Dijkstra d(path, map);
	vector<Vertex> path_simplifie = d.getBestPath(start, goal);
		    
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    

    /*for(size_t i = 0; i < tree.size(); i++)
    {
        cv::Point p(tree.at(i).getPosPix()[0], tree.at(i).getPosPix()[1]);
        cv::circle(image, p, 10, cv::Scalar(255, 255, 0), -1);

        // On relie le nouveau point à son parent
        if (tree.at(i).getParentInd() >= 0)
        {
            cv::line(image, cv::Point(tree.at(i).getPosPix()[0], tree.at(i).getPosPix()[1]), cv::Point(tree.at(tree.at(i).getParentInd()).getPosPix()[0], tree.at(tree.at(i).getParentInd()).getPosPix()[1]), cv::Scalar(0, 0, 0), 1);
        }
        cv::resize(image, outImage, cv::Size(image.cols * 0.7, image.rows * 0.7), 0, 0, CV_INTER_LINEAR);
        imshow("Display Image", outImage);
        cv::waitKey(100);
    }*/

    for(size_t i = 0; i < path.size(); i++)
    {
        cv::Point p(path.at(i).getPosPix()[0], path.at(i).getPosPix()[1]);
        cv::circle(image, p, 10, cv::Scalar(0, 255, 0), -1);

        // On relie les points entre eux
        if (i > 0)
        {
            cv::Point p1(path.at(i-1).getPosPix()[0], path.at(i-1).getPosPix()[1]);
            cv::Point p2(path.at(i).getPosPix()[0], path.at(i).getPosPix()[1]);
            cv::line(image, p1, p2, cv::Scalar(50, 50, 0), 1);
        }
    }

    for(size_t i = 0; i < path_simplifie.size(); i++)
    {
        cv::Point p(path_simplifie.at(i).getPosPix()[0], path_simplifie.at(i).getPosPix()[1]);
        cv::circle(image, p, 10, cv::Scalar(255, 0, 0), -1);

        // On relie les points entre eux
        if (i > 0)
        {
            cv::Point p1(path_simplifie.at(i-1).getPosPix()[0], path_simplifie.at(i-1).getPosPix()[1]);
            cv::Point p2(path_simplifie.at(i).getPosPix()[0], path_simplifie.at(i).getPosPix()[1]);
            cv::line(image, p1, p2, cv::Scalar(0, 50, 50), 1);
        }
    }

    //outImage = cv::Mat(outImage, cv::Rect(10, 10, 90, 90)); // using a rectangle
    cv::resize(image, outImage, cv::Size(image.cols * 0.7, image.rows * 0.7), 0, 0, CV_INTER_LINEAR);
    imshow("Display Image", outImage);
    cv::waitKey();
    cv::destroyAllWindows();


    // Conversion px/m
    nav_msgs::Path real_path;
    for(size_t i = 0; i < path_simplifie.size(); i++)
    {
        geometry_msgs::PoseStamped pt;
        
        pt.pose.position.x = path_simplifie.at(i).getPosPix()[0] * map.info.resolution + map.info.origin.position.x;
        pt.pose.position.y = path_simplifie.at(i).getPosPix()[1] * map.info.resolution + map.info.origin.position.y;
		pt.pose.position.z = map.info.origin.position.z;
        real_path.poses.push_back(pt);
    }
    res.path = real_path;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_map");
    srand(time(NULL));

    ros::NodeHandle n;
    ros::ServiceServer plan = n.advertiseService("plan_srv", planning_function);
    

    ros::spin();

    return 0;
}
