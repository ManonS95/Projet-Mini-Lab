#ifndef VERTEX_HPP
#define VERTEX_HPP

#define DELTA_Q 14

#include <nav_msgs/OccupancyGrid.h>

class Vertex {
	public :
		// Constructor
		Vertex(int x, int y, int parent);

		// Get
		int* getPosPix();
		int getParentInd();
		
		// Set
		void setParentInd(int parent);

		// Methods
		friend bool newConfig(const Vertex &q_near, const Vertex &q, Vertex &q_new, const nav_msgs::OccupancyGrid &map);
		double dist(const Vertex &q);
		
		// Operators
		bool operator==(const Vertex &q);
		
	private :
		int pos_pix[2];
		int parent_ind;
};

Vertex randVertex(int width_pix, int height_pix);
bool newConfig(const Vertex &q_near, const Vertex &q, Vertex &q_new, const nav_msgs::OccupancyGrid &map);
bool freePath(const Vertex &q_last, const Vertex &q_goal, const nav_msgs::OccupancyGrid &map);

#endif //VERTEX_HPP