#include "commande.hpp"

#define PI 3.141592

using namespace std;

Commande::Commande() : pid(1.0, 0.0, 0.0, PI/2, 0.01), threshold(0.1), u1(0.2), ind(0), l1(1)
{}

Commande::Commande(nav_msgs::Path path) : threshold(0.1), u1(0.2), pid(1, 0, 0, PI/2, 0.01), ind(0), l1(1)
{
    init(path);
}

double Commande::theta_error(double theta)
{
	size_t n = ind;
	if(n >= path.poses.size())
	{
		n = path.poses.size()-2;
	}

	double y = path.poses.at(n+1).pose.position.y - path.poses.at(n).pose.position.y;
	double x = path.poses.at(n+1).pose.position.x - path.poses.at(n).pose.position.x;

	double theta_c = atan2(y, x);
  double theta_e = theta - theta_c;
	return atan2(sin(theta_e), cos(theta_e));
}

double Commande::distance(double x, double y)
{
  	size_t n = ind;
  	if(n >= path.poses.size())
  	{
  		n = path.poses.size()-2;
  	}
  	double t_y = path.poses.at(n+1).pose.position.y - path.poses.at(n).pose.position.y;
  	double t_x = path.poses.at(n+1).pose.position.x - path.poses.at(n).pose.position.x;
    double t_norm = sqrt(t_x * t_x + t_y * t_y);

    double n_x = - t_y / t_norm;
    double n_y = t_x / t_norm;

    double p_x = x - path.poses.at(n).pose.position.x;
    double p_y = y - path.poses.at(n).pose.position.y;

    double d = n_x * p_x + n_y * p_y;
    cout << d << endl;
    return d;
}

void Commande::init(const nav_msgs::Path& path)
{
    this->path = path;
    ind = 0;
	for (int i = 0; i < path.poses.size(); i++)
	{
		cout << "x = " << path.poses.at(i).pose.position.x  << " y = " << path.poses.at(i).pose.position.y << endl;
	}
}

double Commande::K(double d, double theta_e)
{
	double k = 1; // Coefficient Proportionnel
	return abs(k * d * cos(theta_e));
}

double Commande::mot_command(double x, double y, double theta)
{
	double theta_e = theta_error(theta);
	double d = distance(x, y);
	return -(u1 * tan(theta_e)) / l1 - (K(d, theta_e) * d) / cos(theta_e);
}

vector<double> Commande::command_law(double x, double y, double theta)
{
	vector<double> u(2, 0);
	double u2 = 0;
	double theta_e = theta_error(theta);

	if(abs(theta_e) < PI/2-0.1)
	{
		u2 = mot_command(x, y, theta);
    u1 = 0.5;
	}
	else
	{
		u2 = pid.correcteur(theta_e);
    u1 = 0;
	}

	if(verification(x, y))
	{
		ind++;
	}
	u.at(0) = u1;
	u.at(1) = u2;
	return u;
}

bool Commande::verification(double x, double y)
{
  	if(ind+1 >= path.poses.size())
  	{
  		return false;
  	}
  	double p_x = x - path.poses.at(ind+1).pose.position.x;
    double p_y = y - path.poses.at(ind+1).pose.position.y;

    double n_x = path.poses.at(ind).pose.position.x - path.poses.at(ind+1).pose.position.x;
    double n_y = path.poses.at(ind).pose.position.y - path.poses.at(ind+1).pose.position.y;

    double scal = p_x * n_x + p_y * n_y;

    if(scal < 0)
    {
    	return true;
    }
    else
    {
    	return false;
    }
}

bool Commande::fin()
{
	if(ind+1 >= path.poses.size())
	{
		return true;
	}
	else
	{
		return false;
	}
}
