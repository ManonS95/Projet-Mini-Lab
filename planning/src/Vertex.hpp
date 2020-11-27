#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <vector>
#include <memory>

class Vertex {
	public :
		// Constructor
		Vertex(int x, int y, std::weak_ptr<Vertex> parent);

		// Get
		int* getPosPix();
		std::weak_ptr<Vertex> getParent();
		
		// Set
		void setParent(std::weak_ptr<Vertex> parent);

		// Method
		bool newConfig(std::weak_ptr<Vertex> q, std::weak_ptr<Vertex> &q_new /**, class MAP map */);
		
	private :
		int pos_pix[2];
		std::weak_ptr<Vertex> parent;
};

Vertex randVertex(int width_pix, int height_pix);

#endif //VERTEX_HPP
