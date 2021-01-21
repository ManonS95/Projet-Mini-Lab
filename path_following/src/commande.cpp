#include "commande.hpp"

Commande::Commande() : threshold(0.1), speed(1)
{
	pid = Pid(1, 0, 0, 0, dt);
}

Commande::Commande(Path path) : threshold(0.1), speed(1)
{
	pid = Pid(1, 0, 0, 0, dt);
    init(path);
}

double Commande::theta_error(double theta)
{
	size_t n = ind;
	if(n == path.size())
	{
		n--;
	}
	double y = path.at(n+1).y - path.at(n).y;
	double x = path.at(n+1).x - path.at(n).x;
	double theta_c = atan2(y, x);
	return theta_c - theta;
}

double Commande::distance(double x, double y)
{
	size_t n = ind;
	if(n > path.size())
	{
		n = path.size()-1;
	}
	double n_y = path.at(n+1).y - path.at(n).y;
	double n_x = path.at(n+1).x - path.at(n).x;
    double n_norm = (n_x * n_x + n_y * n_y);

    double p_x = x - path.at(n).x;
    double p_y = y - path.at(n).y;

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

double Commande::K(double d, double theta_e)
{
	double k = 10; // Coefficient Proportionnel
	return abs(k*d*cos(theta_e));
}

double Commande::mot_command(double x, double y, double theta)
{
	double theta_e = theta_error(theta);
	double d = distance(x, y);
	return - u1 / cos(theta_e) * (sin(theta_e) + K(d,theta_e) * d);
}

vector<double> Commande::command_law(double x, double y, double theta)
{
	vector<double> u(2,0);
	double u2 = 0;
	double theta_e = theta_error(theta);
	if(abs(theta_e)<pi/2)
	{
		u2 = mot_command(x, y, theta);
	}
	else
	{
		u2 = pid.pid(theta_e);
	}
	if(verification(x, y))
	{
		ind++;	
	}
	u.at(0)=u1;
	u.at(1)=u2;
	return u;	
}

bool verification(double x, double y)
{
	if(ind+1>=path.size())
	{
		return false;
	}
	double n_y = path.at(ind+1).y - y;
	double n_x = path.at(ind+1).x - x;
    double n_norm = sqrt(n_x * n_x + n_y * n_y);
    if(n_norm<threshold)
    {
    	return true;
    }
    else
    {
    	return false;
    }
}

bool fin()
{
	if(ind>path.size()-1)
	{
		return true;
	}
	else
	{
		return false;
	}
}
