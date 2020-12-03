#include "Tree.hpp"

// Constructor
Tree::Tree(const Vertex &q)
{
	this->vect_v.push_back(q);
}

// Get
int Tree::getIndex(const Vertex &q)
{
	int index = -1;
	auto it = find(this->vect_v.begin(), this->vect_v.end(), q);
	if(it != this->vect_v.end())
	{
		index = it - this->vect_v.begin();
	}
	return index;
}

Vertex Vertex::getLast()
{
	return this->vect_v.back();
}

// Method
Return Tree::extend(const Vertex &q, const nav_msgs::OccupancyGrid &map)
{
	Vertex q_near = this->nearestNeighbor(q);
	Vertex q_new;
	Return res;
	if (newConfig(q_near, q, q_new, map))
	{
		q_new.setParentInd(this->getIndex(q_near));
		this->addVertex(q_new);
		if (q_new == q)
		{
			res = Return::Reached;
		}
		else
		{
			res = Return::Advanced;
		}
	}
	else
	{
		res = Return::Trapped;
	}
	return res;
}

Vertex Tree::nearestNeighbor(const Vertex &q)
{
	Vertex q_near = vect_v.at(0);
	double dist_min = q_near.dist(q);
	double dist_temp;

	for(int i = 1; i < vect_v.size(); i++)
	{
		dist_temp = vect_v.at(i).dist(q);
		if(dist_temp < dist_min)
		{
			q_near = vect_v.at(i);
			dist_min = dist_temp;
		}
	}

	return q_near;
}

void Tree::addVertex(const Vertex &q)
{
	this->vect_v.push_back(q);
}

Tree build_rrt(const Vertex &q_start, const Vertex &q_goal, const nav_msgs::OccupancyGrid &map)
{
	Vertex q_rand, q_last;
	Tree t = Tree(q_start);
	
	do
	{
		q_rand = randVertex(map.info.width, map.info.height);
		t.extend(q_rand, map);
		q_last = t.getLast();
	} while (!freePath(q_last, q_goal, map));

	q_goal.setParentInd(t.getIndex(q_last));
	t.addVertex(q_goal);

	return t;
}

