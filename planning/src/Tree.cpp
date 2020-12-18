#include <stdexcept>
#include <algorithm>
#include "Tree.hpp"

using namespace std;

// Constructor
Tree::Tree(const Vertex &q)
{
	this->vect_v.push_back(q);
}

Tree::Tree() {}

// Get
int Tree::getIndex(const Vertex &q)
{
	int index = -1;
	auto it = find(this->vect_v.begin(), this->vect_v.end(), q);
	if(it != this->vect_v.end())
	{
		index = it - this->vect_v.begin();
	}
	else
	{
		throw logic_error("q not in graph.");
	}
	return index;
}

Vertex Tree::getLast()
{
	return this->vect_v.back();
}

// Method
Return Tree::extend(const Vertex &q, const nav_msgs::OccupancyGrid &map, vector<Vertex> &t_r)
{
	Vertex q_near = this->nearestNeighbor(q);
	Vertex q_new;
	Return res;
	if (q_near.newConfig(q, q_new, map))
	{
		int ind;
		try
		{
			ind = this->getIndex(q_near);
		}
		catch (logic_error& e)
		{
			cerr << e.what() << endl;
			exit(-1);
		}
		
		q_new.setParentInd(ind);
		this->addVertex(q_new);
		
		if (q_new == q)
		{
			res = Return::Reached;
		}
		else
		{
			res = Return::Advanced;
		}
		t_r.push_back(q);
	}
	else
	{
		res = Return::Trapped;
		cout<<"Point non retenu !"<<endl;		
	}
	return res;
}

vector<Vertex> Tree::getTree()
{
	return this->vect_v;
}

vector<Vertex> Tree::getPath(const Vertex &q_goal)
{
	Vertex q = q_goal;
	vector<Vertex> path = vector<Vertex>();
	int ind;
	try
	{
		ind = this->getIndex(q);
	}
	catch (logic_error& e)
	{
		cerr << e.what() << endl;
		exit(-1);
	}
	while(ind>-1)
	{
		q = this->vect_v.at(ind);
		path.push_back(q);
		ind = q.getParentInd();
	}

	reverse(path.begin(), path.end());
	return path;
}

Vertex Tree::nearestNeighbor(const Vertex &q)
{
	Vertex q_near = vect_v.at(0);
	double dist_min = q_near.dist(q);
	double dist_temp;

	for(int i = 1; i < vect_v.size(); i++)
	{
		dist_temp = vect_v.at(i).dist(q);
		cout<<"dist_temp = "<<dist_temp<<" dist_min = "<<dist_min<<endl;
		if(dist_temp < dist_min)
		{
			q_near = vect_v.at(i);
			dist_min = dist_temp;
		}
	}
	cout<<"Distance_min = "<<dist_min<<endl;
	return q_near;
}

void Tree::addVertex(const Vertex &q)
{
	this->vect_v.push_back(q);
}

Tree build_rrt(const Vertex &q_start, Vertex &q_goal, const nav_msgs::OccupancyGrid &map, vector<Vertex> &t_r)
{
	Vertex q_rand, q_last;
	Tree t = Tree(q_start);
	int i = 0;
	do
	{
		q_rand = randVertex(map.info.width, map.info.height);
		t.extend(q_rand, map, t_r);
		q_last = t.getLast();
		i++;
	} while (!q_last.freePath(q_goal, map) and i < LIMITS);

	int ind;
	try
	{
		ind = t.getIndex(q_last);
	}
	catch (logic_error& e)
	{
		cerr << e.what() << endl;
		exit(-1);
	}
	cout<<"ind = "<<ind<<endl;
	q_goal.setParentInd(ind);
	t.addVertex(q_goal);

	return t;
}

