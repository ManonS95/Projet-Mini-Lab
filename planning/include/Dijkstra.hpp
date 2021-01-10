#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <vector>
#include <nav_msgs/OccupancyGrid.h>

#include "Vertex.hpp"

class Dijkstra {
	// https://fr.wikipedia.org/wiki/Algorithme_de_Dijkstra#:~:text=En%20th%C3%A9orie%20des%20graphes%2C%20l,probl%C3%A8me%20du%20plus%20court%20chemin.
    public :
		// Constructor
		Dijkstra(std::vector<Vertex> v, const nav_msgs::OccupancyGrid &map);
		
		// Get
		std::vector<Vertex> getBestPath(Vertex begin, Vertex end);
		
	private :
		std::vector<Vertex> G;
		std::vector<double> dist;
        std::vector<int> path;
		nav_msgs::OccupancyGrid map;
		
        void initialisation(size_t v);
        size_t find_min(std::vector<size_t> Q);
        void update_dist(size_t ind1, size_t ind2);
        void apply_dijkstra(size_t begin, size_t end);
};

#endif // DIJKSTRA_HPP
