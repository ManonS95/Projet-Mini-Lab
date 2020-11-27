#include "Vertex.hpp"
#include <cstdlib>
#include <ctime>

using namespace std;

Vertex::Vertex(int x, int y, std::weak_ptr<Vertex> parent)
{
	this->pos_pix[0] = x;
	this->pos_pix[1] = y;
	this->parent = parent;
}

int* Vertex::getPosPix()
{
	return this->pos_pix;
}

std::weak_ptr<Vertex> Vertex::getParent()
{
	return this->parent;
}

void Vertex::setParent(std::weak_ptr<Vertex> parent)
{
	this->parent = parent;
}

Vertex randVertex(int width_pix, int height_pix)
{
	srand(time(NULL));
	weak_ptr<Vertex> v;
	int x = rand()%width_pix;
	int y = rand()%height_pix;
	return Vertex(x, y, v);
}
