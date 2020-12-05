#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <nav_msgs/OccupancyGrid.h>

#define DELTA_Q 14
#define  STRECH 50

class Vertex {
	public :
		// Constructor
		Vertex(int x, int y, int parent);
		Vertex(int x, int y);
		
		// Get
		int* getPosPix();
		int getParentInd();
		
		// Set
		void setParentInd(int parent);

		// Methods
		double dist(const Vertex &q);
		bool hasParent();
		bool newConfig(const Vertex &q, Vertex &q_new, const nav_msgs::OccupancyGrid &map);
		bool freePath(const Vertex &q_goal, const nav_msgs::OccupancyGrid &map);
		
		// Operators
		bool operator==(const Vertex &q);
		
	private :
		int pos_pix[2];
		int parent_ind;
};

Vertex randVertex(int width_pix, int height_pix);
bool ifFree(int x, int y, const nav_msgs::OccupancyGrid &map);

#endif //VERTEX_HPP
