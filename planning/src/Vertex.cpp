#include "Vertex.hpp"
#include <cstdlib>
#include <ctime>
#include <cmath>

using namespace std;

Vertex::Vertex(int x, int y, int parent)
{
	this->pos_pix[0] = x;
	this->pos_pix[1] = y;
	this->parent_ind = parent;
}

int* Vertex::getPosPix()
{
	return this->pos_pix;
}

int Vertex::getParentInd()
{
	return this->parent_ind;
}

void Vertex::setParentInd(int parent)
{
	this->parent_ind = parent;
}

double Vectex::dist(const Vertex &q)
{
	return sqrt((this->pos_pix[0] - q.pos_pix[0]) * (this->pos_pix[0] - q.pos_pix[0]) + (this->pos_pix[1] - q.pos_pix[1]) * (this->pos_pix[1] - q.pos_pix[1]));
}

Vertex randVertex(int width_pix, int height_pix)
{
	srand(time(NULL));
	int x = rand()%width_pix;
	int y = rand()%height_pix;
	return Vertex(x, y, -1);
}

bool newConfig(const Vertex &q_near, const Vertex &q, Vertex &q_new, const nav_msgs::OccupancyGrid &map)
{
	double d_x = (q_near.pos_pix[0] - q.pos_pix[0]);
	double d_y = (q_near.pos_pix[1] - q.pos_pix[1]);
	double d = sqrt(d_x * d_x + d_y * d_y);
	
	// Implémenter q_new avec delta_q
	if (d <= DELTA_Q)
	{
		q_new = q;
	}
	else
	{
		double r = DELTA_Q / d;
		q_new.pos_pix[0] = q_near.pos_pix[0] + (int)(r * d_x);
		q_new.pos_pix[1] = q_near.pos_pix[1] + (int)(r * d_y);
	}

	// Si q_new est validé
	int i = map.info.width * q_new.pos_pix[1] + q_new.pos_pix[0]
	if (!freePath(q_near, q_new, map))
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool freePath(const Vertex &q_last, const Vertex &q_goal, const nav_msgs::OccupancyGrid &map)
{

}

// Operators
bool Vertex::operator==(const Vertex &q)
{
	if(this->pos_pix == q->pos_pix)
	{
		return true;
	}
	return false;
}