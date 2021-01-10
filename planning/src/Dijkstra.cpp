#include "Dijkstra.hpp"
#include <limits>
#include <algorithm>

using namespace std;

#define INFINITY_BY_THE_WAY numeric_limits<double>::max()


Dijkstra::Dijkstra(vector<Vertex> v, const nav_msgs::OccupancyGrid &map) : G(v), path(), map(map)
{
    this->initialisation(0);
}

void Dijkstra::initialisation(size_t v)
{
    dist.resize(this->G.size());
	path.resize(G.size());

    for (size_t i = 0; i < dist.size(); i++)
    {
        dist.at(i) = (i == v)?0:INFINITY_BY_THE_WAY;
        path.at(i) = -1;
    }
}

size_t Dijkstra::find_min(vector<size_t> Q)
{
    double mini = INFINITY_BY_THE_WAY;
    size_t vertex = -1;

    for(size_t i = 0; i < Q.size(); i++)
    {
        if (dist.at(Q.at(i)) < mini)
        {
            mini = dist.at(Q.at(i));
            vertex = Q.at(i);
        }
    }
    return vertex;
}

void Dijkstra::update_dist(size_t ind1, size_t ind2)
{
    if (dist.at(ind2) > dist.at(ind1) + G.at(ind1).dist(G.at(ind2)))
    {
        dist.at(ind2) = dist.at(ind1) + G.at(ind1).dist(G.at(ind2));
        path.at(ind2) = ind1;
    }
}

void Dijkstra::apply_dijkstra(size_t begin, size_t end)
{
	initialisation(begin);
	vector<size_t> Q;
	for(size_t i = 0; i< G.size(); i++)
	{
		Q.push_back(i);
	}
	while(Q.size()>0)
	{
		size_t s1 = find_min(Q);
		Q.erase(remove(Q.begin(), Q.end(), s1), Q.end());

		for(size_t i=0; i<Q.size(); i++)
		{
			size_t s2 = Q.at(i);
			
			if(G.at(s1).freePath(G.at(s2), map))
			{
				update_dist(s1, s2);
			}
		}
	}
}

vector<Vertex> Dijkstra::getBestPath(Vertex begin, Vertex end)
{
	size_t ind_beg = 0;
	size_t ind_end = 0;
	for(size_t i = 0; i < G.size(); i++)
	{
		if(G.at(i) == begin)
		{
			ind_beg = i;
		}
		if(G.at(i) == end)
		{
			ind_end = i;
		}
	}
	apply_dijkstra(ind_beg, ind_end);
	vector<Vertex> A;
	size_t s = ind_end;
	while(s!= ind_beg)
	{
		A.push_back(G.at(s));
		s = path.at(s);
	}
	A.push_back(G.at(s));
	reverse(A.begin(), A.end());
	return A;
}
