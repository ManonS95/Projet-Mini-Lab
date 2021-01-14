#include "commande.hpp"

#include <cmath>

Commande::Commande(Path path)
{
	init(path);
}

double Commande::theta_error(Point current_pos)
{
	size_t n = ind;
	if(n == path.size())
	{
		n--;
	}
	double y = path.at(n+1).y - path.at(n).y;
	double x = path.at(n+1).x - path.at(n).x;
	double theta_c = atan2(y, x);
	double theta = current_pos.theta.EULER.Z;
	return theta_c - theta;
}

double Commande::distance(Point current_pos)
{
	
}
