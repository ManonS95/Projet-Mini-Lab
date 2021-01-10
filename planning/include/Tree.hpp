#ifndef TREE_HPP
#define TREE_HPP

#include <vector>
#include "Vertex.hpp"
#include <nav_msgs/OccupancyGrid.h>


#define LIMITS 1000

/**
* Implementation inspired by : 
* http://www.kuffner.org/james/papers/kuffner_icra2000.pdf
*/

enum class Return { Reached, Advanced, Trapped }; // Change name

class Tree
{
	public :
		// Constructor
		Tree(const Vertex &q);
		Tree();

		// Get
		int getIndex(const Vertex &q) const;
		Vertex getLast() const;
		std::vector<Vertex> getTree() const;
		std::vector<Vertex> getPath(const Vertex &q_goal) const;
		std::vector<Vertex> getPath(const Tree &t_goal) const;
		
		// Method
		Return extend(const Vertex &q, const nav_msgs::OccupancyGrid &map);
		Return connect(const Vertex &q, const nav_msgs::OccupancyGrid &map);
		Vertex nearestNeighbor(const Vertex &q) const;
		void addVertex(const Vertex &q);
		
	private :
		std::vector<Vertex> vect_v;
};

Tree build_rrt(const Vertex &q_start, Vertex &q_goal, const nav_msgs::OccupancyGrid &map);
std::vector<Vertex> rrt_connect_planner(const Vertex &q_start, Vertex &q_goal, const nav_msgs::OccupancyGrid &map);

#endif // TREE_HPP
