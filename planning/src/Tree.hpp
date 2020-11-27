#ifndef TREE_HPP
#define TREE_HPP

#include <list>
#include <memory>
#include "Vertex.hpp"


/**
* Implementation inspired by : 
* http://www.kuffner.org/james/papers/kuffner_icra2000.pdf
*/

enum class Return { Reached, Advanced, Trapped }; // Change name

class Tree
{
	public :
		// Constructor
		Tree(std::weak_ptr<Vertex> q);
		
		// Method
		Return extend(std::weak_ptr<Vertex> q /**, Class MAP map **/);
		std::weak_ptr<Vertex> nearestNeighbor(std::weak_ptr<Vertex> q);
		void addVertex(std::weak_ptr<Vertex> q);
		
	private :
		std::list<std::weak_ptr<Vertex>> list_v;
};

Tree build_rrt(std::weak_ptr<Vertex> q_init /**, Class MAP map **/);

#endif // TREE_HPP
