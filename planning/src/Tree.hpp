#ifndef TREE_HPP
#define TREE_HPP

#include <vector>
#include "Vertex.hpp"
#include <nav_msgs/OccupancyGrid.h>


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

		// Get
		int getIndex(const Vertex &q);
		
		// Method
		Return extend(const Vertex &q, const nav_msgs::OccupancyGrid &map);
		Vertex nearestNeighbor(const Vertex &q);
		void addVertex(const Vertex &q);
		
	private :
		std::vector<Vertex> vect_v;
};

Tree build_rrt(const Vertex &q_start, const Vertex &q_goal, const nav_msgs::OccupancyGrid &map);

#endif // TREE_HPP
