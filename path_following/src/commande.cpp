#include "commande.hpp"

#include <cmath>

Commande::Commande(Path path) : threshold(0.1)
{
	pid = Pid(1, 0, 0, 0);
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
	size_t n = ind;
	if(n == path.size())
	{
		n--;
	}
	double n_y = path.at(n+1).y - path.at(n).y;
	double n_x = path.at(n+1).x - path.at(n).x;
    double n_norm = (n_x * n_x + n_y * n_y);

    double p_x = current_pos.x - path.at(n).x;
    double p_y = current_pos.y - path.at(n).y;

    double p_proj_x = (n_x * p_x + n_y * p_y) * n_x / n_norm;
    double p_proj_y = (n_x * p_x + n_y * p_y) * n_y / n_norm;

    double d_x = p_proj_x - p_x;
    double d_y = p_proj_y - p_y;

    return sqrt(d_x * d_x +  d_y * d_y);
}

void Commande::init(Path path)
{
    this->path = path;
    ind = 0;
}

cmd_vel Commande::following(Point current_pos)
{
    
}