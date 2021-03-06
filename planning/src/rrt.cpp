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

#define RESO_IM 0.35
#define METHODO 1
#define AFFICHAGE 1
#define STATIC 1

using namespace std;

int goal_x, goal_y;

nav_msgs::OccupancyGrid map_dilatation(const nav_msgs::OccupancyGrid &map, int r_robot)
{
    nav_msgs::OccupancyGrid new_map = map;
    int cpt_x;

    for (size_t y = 0; y < map.info.height; y++)
    {
        for (size_t x = 0; x < map.info.width; x++)
        {
            if (map.data[x + map.info.width * y] != 0) // Si le pixel est noir
            {
            	for (int i = -r_robot; i <=r_robot; i++)
                {
                	for (int j = -r_robot; j <=r_robot; j++)
                	{
                		if(!(y+i<0 || x+j<0 || y+i>=map.info.height || x+j>=map.info.width))
                		{
                			new_map.data[(x+j) + new_map.info.width * (y+i)] = 1;
                		}
                	}
               	}
            }
        }
    }

    return new_map;
}

void CallBackFunction(int event, int x, int y, int flags, void* userdata)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        goal_x = x / RESO_IM;
        goal_y = y / RESO_IM;
    }
}

bool planning_function(planning::RRTPlanning::Request& req, planning::RRTPlanning::Response& res)
{
    nav_msgs::OccupancyGrid map;

    map = map_dilatation(req.map, 10);

    cv::Mat image;
    cv::Mat outImage;
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    image = cv::Mat(map.info.height, map.info.width, CV_8UC3, cv::Scalar::all(0));

    if(AFFICHAGE)
    {
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
    }

    Vertex start, goal;
    // Récupérer positions start et goal
    if (STATIC == 0)
    {
      int start_x = round((req.start.translation.x - map.info.origin.position.x) / map.info.resolution);
      int start_y = round((req.start.translation.y - map.info.origin.position.y) / map.info.resolution);
      int goal_x = round((req.goal.translation.x - map.info.origin.position.x) / map.info.resolution);
      int goal_y = round((req.goal.translation.y - map.info.origin.position.y) / map.info.resolution);
      start = Vertex(start_x, start_y);
      goal = Vertex(goal_x, goal_y);
    }
    else
    {
      start = Vertex(1200, 1000);
      goal = Vertex(800, 800);
      cv::Point ps(start.getPosPix()[0], start.getPosPix()[1]);
      cv::Point pg(goal.getPosPix()[0], goal.getPosPix()[1]);
      cv::circle(image, ps, 10, cv::Scalar(0, 0, 255), -1);
      cv::circle(image, pg, 10, cv::Scalar(0, 0, 255), -1);
    }



    vector<Vertex> path;

    if(METHODO == 1) // rrt_build
    {
      Tree t = build_rrt(start, goal, map);
      path = t.getPath(goal);
      vector<Vertex> tree = t.getTree();

      // Affichage
      if(AFFICHAGE)
      {
        for(size_t i = 0; i < tree.size(); i++)
        {
            if (i!= 0)
            {
                cv::Point p(tree.at(i).getPosPix()[0], tree.at(i).getPosPix()[1]);
                cv::circle(image, p, 10, cv::Scalar(255, 255, 0), -1);
            }

            // On relie le nouveau point à son parent
            if (tree.at(i).getParentInd() >= 0)
            {
                cv::line(image, cv::Point(tree.at(i).getPosPix()[0], tree.at(i).getPosPix()[1]), cv::Point(tree.at(tree.at(i).getParentInd()).getPosPix()[0], tree.at(tree.at(i).getParentInd()).getPosPix()[1]), cv::Scalar(0, 0, 0), 1);
            }
            cv::resize(image, outImage, cv::Size(image.cols * 0.7, image.rows * 0.7), 0, 0, CV_INTER_LINEAR);
            imshow("Display Image", outImage);
            cv::waitKey(100);
        }
      }
    }
    else if(METHODO == 2) // rrt_connect
    {
        Tree t_a = Tree(start);
        Tree t_b = Tree(goal);
        path = rrt_connect_planner(start, goal, map, t_a, t_b);
        // Affichage
        if(AFFICHAGE)
        {
            int cpt = 0;
            while (cpt < max(t_a.getTree().size(), t_b.getTree().size()))
            {
                if (t_a.getTree().size() > cpt)
                {
                    if (cpt!= 0)
                    {
                        cv::Point p_a(t_a.getTree().at(cpt).getPosPix()[0], t_a.getTree().at(cpt).getPosPix()[1]);
                        cv::circle(image, p_a, 10, cv::Scalar(255, 255, 0), -1);
                    }
                    // On relie le nouveau point à son parent
                    if (t_a.getTree().at(cpt).getParentInd() >= 0)
                    {
                        cv::line(image, cv::Point(t_a.getTree().at(cpt).getPosPix()[0], t_a.getTree().at(cpt).getPosPix()[1]), cv::Point(t_a.getTree().at(t_a.getTree().at(cpt).getParentInd()).getPosPix()[0], t_a.getTree().at(t_a.getTree().at(cpt).getParentInd()).getPosPix()[1]), cv::Scalar(0, 0, 0), 1);
                    }
                }
                if (t_b.getTree().size() > cpt)
                {
                    if (cpt!= 0)
                    {
                        cv::Point p_b(t_b.getTree().at(cpt).getPosPix()[0], t_b.getTree().at(cpt).getPosPix()[1]);
                        cv::circle(image, p_b, 10, cv::Scalar(255, 0, 255), -1);
                    }
                    // On relie le nouveau point à son parent
                    if (t_b.getTree().at(cpt).getParentInd() >= 0)
                    {
                        cv::line(image, cv::Point(t_b.getTree().at(cpt).getPosPix()[0], t_b.getTree().at(cpt).getPosPix()[1]), cv::Point(t_b.getTree().at(t_b.getTree().at(cpt).getParentInd()).getPosPix()[0], t_b.getTree().at(t_b.getTree().at(cpt).getParentInd()).getPosPix()[1]), cv::Scalar(0, 0, 0), 1);
                    }
                }

                cv::resize(image, outImage, cv::Size(image.cols * 0.7, image.rows * 0.7), 0, 0, CV_INTER_LINEAR);
                imshow("Display Image", outImage);
                cv::waitKey(100);
                cpt++;
            }
        }
    }
  	Dijkstra d(path, map);
  	vector<Vertex> path_simplifie = d.getBestPath(start, goal);

    if(AFFICHAGE)
    {
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

      cv::resize(image, outImage, cv::Size(image.cols * 0.7, image.rows * 0.7), 0, 0, CV_INTER_LINEAR);
      imshow("Display Image", outImage);
      cv::waitKey();
      cv::destroyAllWindows();
    }

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
