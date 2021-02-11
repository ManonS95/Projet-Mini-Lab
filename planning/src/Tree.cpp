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
int Tree::getIndex(const Vertex &q) const
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

Vertex Tree::getLast() const
{
	return this->vect_v.back();
}

// Method
Return Tree::extend(const Vertex &q, const nav_msgs::OccupancyGrid &map)
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
	}
	else
	{
		res = Return::Trapped;
	}
	return res;
}

Return Tree::connect(const Vertex &q, const nav_msgs::OccupancyGrid &map)
{
	Return s;
	do
	{
		s = extend(q, map);
	} while (s == Return::Advanced);
	return s;
}

vector<Vertex> Tree::getTree() const
{
	return this->vect_v;
}

vector<Vertex> Tree::getPath(const Vertex &q_goal) const
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

vector<Vertex> Tree::getPath(const Tree &t_goal) const
{
	vector<Vertex> path_a = this->getPath(this->getLast());
	vector<Vertex> path_b = t_goal.getPath(t_goal.getLast());

	reverse(path_b.begin(), path_b.end());
	path_a.pop_back();
	path_a.insert(path_a.end(), path_b.begin(), path_b.end());

	return path_a;
}

Vertex Tree::nearestNeighbor(const Vertex &q) const
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

Tree build_rrt(const Vertex &q_start, Vertex &q_goal, const nav_msgs::OccupancyGrid &map)
{
	Vertex q_rand, q_last;
	Tree t = Tree(q_start);
	int i = 0;
	do
	{
		q_rand = randVertex(map.info.width, map.info.height);
		if (t.extend(q_rand, map) != Return::Trapped)
		{
			i++;
		}
		q_last = t.getLast();
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
	if(q_last.freePath(q_goal, map))
	{
		q_goal.setParentInd(ind);
		t.addVertex(q_goal);
	}

	return t;
}

std::vector<Vertex> rrt_connect_planner(const Vertex &q_start, Vertex &q_goal, const nav_msgs::OccupancyGrid &map)
{
	Tree t_a = Tree(q_start);
	Tree t_b = Tree(q_goal);
	Vertex q_rand, q_last;

	int i = 0;
	do
	{
		q_rand = randVertex(map.info.width, map.info.height);
		if (t_a.extend(q_rand, map) != Return::Trapped)
		{
			q_last = t_a.getLast();
			i++;
			if (t_b.connect(q_last, map) == Return::Reached)
			{
				if (t_a.getTree().at(0) == q_start)
				{
					return t_a.getPath(t_b);
				}
				else
				{
					return t_b.getPath(t_a);
				}
			}
		}
		swap(t_a, t_b);

	} while (i < LIMITS);
	cout<<"Path not found -> Limit too short!"<<endl;
	exit(1);
}
