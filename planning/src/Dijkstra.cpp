#include "Dijkstra.hpp"

using namespace std;


Dijkstra::Dijkstra(vector<Vertex> v) : G(v), path()
{
    this->initialisation(0);
}

void Dijkstra::initialisation(size_t v)
{
    this->dist.resize(this->G.size());

    for (size_t i = 0; i < dist.size(); i++)
    {
        dist.at(i) = (i == v)?0:-1;
    }
}

size_t Dijkstra::find_min(vector<size_t> Q)
{
    size_t mini = -1;
    int vertex = -1;

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
        path
    }
}