#include "Vertex.hpp"
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <random>

using namespace std;

Vertex::Vertex(int x, int y, int parent)
{
	this->pos_pix[0] = x;
	this->pos_pix[1] = y;
	this->parent_ind = parent;
}

Vertex::Vertex(int x, int y)
{
	this->pos_pix[0] = x;
	this->pos_pix[1] = y;
	this->parent_ind = -1;
}

Vertex::Vertex()
{
	this->pos_pix[0] = 0;
	this->pos_pix[1] = 0;
	this->parent_ind = -1;
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

double Vertex::dist(const Vertex &q)
{
	int d_x = (this->pos_pix[0] - q.pos_pix[0]);
	int d_y = (this->pos_pix[1] - q.pos_pix[1]);
	double d = sqrt(d_x * d_x + d_y * d_y);
	return d;
}

bool Vertex::hasParent()
{
	return this->parent_ind>-1;
}

Vertex randVertex(int width_pix, int height_pix)
{
	std::random_device generator;
	std::uniform_int_distribution<int> distribution_x(0, width_pix);
	std::uniform_int_distribution<int> distribution_y(0, height_pix);
	int x = distribution_x(generator);
	int y = distribution_y(generator);
	return Vertex(x, y, -1);
}

bool Vertex::newConfig(const Vertex &q, Vertex &q_new, const nav_msgs::OccupancyGrid &map)
{
	int d_x = (this->pos_pix[0] - q.pos_pix[0]);
	int d_y = (this->pos_pix[1] - q.pos_pix[1]);
	double d = sqrt(d_x * d_x + d_y * d_y);
	
	// Implémenter q_new avec delta_q
	if (d <= DELTA_Q)
	{
		q_new = q;
	}
	else
	{
		double r = DELTA_Q / d;
		q_new.pos_pix[0] = this->pos_pix[0] + (int)(r * d_x);
		q_new.pos_pix[1] = this->pos_pix[1] + (int)(r * d_y);
	}

	// Si q_new est validé
	if (!this->freePath(q_new, map))
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool Vertex::freePath(const Vertex &q_goal, const nav_msgs::OccupancyGrid &map)
{
	// https://fr.wikipedia.org/wiki/Algorithme_de_trac%C3%A9_de_segment_de_Bresenham
	int x = this->pos_pix[0];
	int y = this->pos_pix[1];
	int d_x = q_goal.pos_pix[0]-x;
	int d_y = q_goal.pos_pix[1]-y;
	if(d_x!=0)
	{
		if(d_x>0)
		{
			if(d_y!=0)
			{
				if(d_y>0) // Vecteur oblique dans le 1er cadran
				{
					if(d_x>=d_y) // Vecteur diagonal ou oblique proche de l'horizontale, dans le 1er octant
					{
						int e = d_x;
						d_x *= 2;
						d_y *= 2;
						do
						{
							if(!isFree(x, y, map))
							{
								return false;
							}
							x++;
							e-=d_y;
							if(e<0)
							{
								y++;
								e+=d_x;
							}
						}while(x != q_goal.pos_pix[0]);
					}
					else // Vecteur oblique proche de la verticale, dans le 2e octant
					{
						int e = d_y;
						d_x *= 2;
						d_y *= 2;
						do
						{
							if(!isFree(x, y, map))
							{
								return false;
							}
							y++;
							e-=d_x;
							if(e<0)
							{
								x++;
								e+=d_y;
							}
						}while(y != q_goal.pos_pix[1]);	
					}
				}
				else // Vecteur oblique dans le 4e cadran
				{
					if(d_x>=-d_y) // Vecteur diagonal ou oblique proche de l'horizontale, dans le 8e octant
					{
						int e = d_x;
						d_x *= 2;
						d_y *= 2;
						do
						{
							if(!isFree(x, y, map))
							{
								return false;
							}
							x++;
							e+=d_y;
							if(e<0)
							{
								y--;
								e+=d_x;
							}
						}while(x != q_goal.pos_pix[0]);
					}
					else // Vecteur oblique proche de la verticale, dans le 7e octant
					{
						int e = d_y; // e < 0
						d_x *= 2;
						d_y *= 2;
						do
						{
							if(!isFree(x, y, map))
							{
								return false;
							}
							y--;
							e+=d_x;
							if(e>0)
							{
								x++;
								e+=d_y;
							}
						}while(y != q_goal.pos_pix[1]);	
					}
				}
			}
			else // Vecteur horizontal vers la droite
			{
				do
				{
					if(!isFree(x, y, map))
					{
						return false;
					}
					x++;
				}while(x != q_goal.pos_pix[0]);	
			}
		}
		else // d_x < 0
		{
			if(d_y!=0)
			{
				if(d_y>0) // Vecteur oblique dans le 2e cadran
				{
					if(-d_x>=d_y) // Vecteur diagonal ou oblique proche de l'horizontale, dans le 4e octant
					{
						int e = d_x; // e < 0
						d_x *= 2;
						d_y *= 2;
						do
						{
							if(!isFree(x, y, map))
							{
								return false;
							}
							x--;
							e+=d_y;
							if(e>=0)
							{
								y++;
								e+=d_x;
							}
						}while(x != q_goal.pos_pix[0]);
					}
					else // Vecteur oblique proche de la verticale, dans le 3e octant
					{
						int e = d_y;
						d_x *= 2;
						d_y *= 2;
						do
						{
							if(!isFree(x, y, map))
							{
								return false;
							}
							y++;
							e+=d_x;
							if(e<=0)
							{
								x--;
								e+=d_y;
							}
						}while(y != q_goal.pos_pix[1]);	
					}
				}
				else // Vecteur oblique dans le 3e cadran
				{
					if(d_x<=d_y) // Vecteur diagonal ou oblique proche de l'horizontale, dans le 5e octant
					{
						int e = d_x; // e < 0
						d_x *= 2;
						d_y *= 2;
						do
						{
							if(!isFree(x, y, map))
							{
								return false;
							}
							x--;
							e-=d_y;
							if(e>=0)
							{
								y--;
								e+=d_x;
							}
						}while(x != q_goal.pos_pix[0]);
					}
					else // Vecteur oblique proche de la verticale, dans le 6e octant
					{
						int e = d_y; // e < 0
						d_x *= 2;
						d_y *= 2;
						do
						{
							if(!isFree(x, y, map))
							{
								return false;
							}
							y--;
							e-=d_x;
							if(e>=0)
							{
								x--;
								e+=d_y;
							}
						}while(y != q_goal.pos_pix[1]);	
					}
				}
			}
			else // Vecteur horizontal vers la gauche
			{
				do
				{
					if(!isFree(x, y, map))
					{
						return false;
					}
					x--;
				}while(x != q_goal.pos_pix[0]);	
			}
		}
	}
	else // d_x == 0
	{
		if(d_y!=0) // Vecteur vertical croissant
		{
			if(d_y>0)
			{
				do
				{
					if(!isFree(x, y, map))
					{
						return false;
					}
					y++;
				}while(y != q_goal.pos_pix[1]);
			}
			else // Vecteur vertical decroissant
			{
				do
				{
					if(!isFree(x, y, map))
					{
						return false;
					}
					y++;
				}while(y != q_goal.pos_pix[1]);
			}
		}
	}
 	return isFree(q_goal.pos_pix[0], q_goal.pos_pix[1], map);
}

// Operators
bool Vertex::operator==(const Vertex &q)
{
	if((this->pos_pix[0] == q.pos_pix[0]) and (this->pos_pix[1] == q.pos_pix[1]))
	{
		return true;
	}
	return false;
}

bool isFree(int x, int y, const nav_msgs::OccupancyGrid &map)
{
	int path = map.data[map.info.width*y+x];
	if(path >= STRECH)
	{
		return false;
	}
	return true;
}
