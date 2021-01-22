#include "commande.hpp"

 #define PI 3.141592 

using namespace std;

Commande::Commande() : pid(1.0, 0.0, 0.0, 0.0, 0.01), threshold(0.1), u1(1)
{}

Commande::Commande(nav_msgs::Path path) : threshold(0.1), u1(1), pid(1, 0, 0, 0, 0.01)
{
    init(path);
}

double Commande::theta_error(double theta)
{
	size_t n = ind;
	cout << path.poses.size() << endl;
	if(n >= path.poses.size())
	{
		n = path.poses.size()-2;
		cout << n << endl;
	}
	cout<<"theta_error1"<<endl;
	double y = path.poses.at(n+1).pose.position.y - path.poses.at(n).pose.position.y;
	double x = path.poses.at(n+1).pose.position.x - path.poses.at(n).pose.position.x;
	cout<<"theta_error2"<<endl;
	double theta_c = atan2(y, x);
	return theta_c - theta;
}

double Commande::distance(double x, double y)
{
	size_t n = ind;
	if(n >= path.poses.size())
	{
		n = path.poses.size()-2;
	}
	double n_y = path.poses.at(n+1).pose.position.y - path.poses.at(n).pose.position.y;
	double n_x = path.poses.at(n+1).pose.position.x - path.poses.at(n).pose.position.x;
    double n_norm = (n_x * n_x + n_y * n_y);

    double p_x = x - path.poses.at(n).pose.position.x;
    double p_y = y - path.poses.at(n).pose.position.y;

    double p_proj_x = (n_x * p_x + n_y * p_y) * n_x / n_norm;
    double p_proj_y = (n_x * p_x + n_y * p_y) * n_y / n_norm;

    double d_x = p_proj_x - p_x;
    double d_y = p_proj_y - p_y;

    return sqrt(d_x * d_x +  d_y * d_y);
}

void Commande::init(const nav_msgs::Path& path)
{
    this->path = path;
    ind = 0;
	cout << "init" << endl;
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
	vector<double> u(2, 0);
	double u2 = 0;
	cout<<"Commande_law1"<<endl;
	double theta_e = theta_error(theta);
	cout<<"Commande_law2"<<endl;
	if(abs(theta_e) < PI/2)
	{
		cout<<"Commande_law mot"<<endl;
		u2 = mot_command(x, y, theta);
	}
	else
	{
		cout<<"Commande_law corr"<<endl;
		u2 = pid.correcteur(theta_e);
	}
	cout<<"Commande_law3"<<endl;
	if(verification(x, y))
	{
		ind++;	
	}
	u.at(0)=u1;
	u.at(1)=u2;
	return u;	
}

bool Commande::verification(double x, double y)
{
	if(ind+1>=path.poses.size())
	{
		return false;
	}
	double n_y = path.poses.at(ind+1).pose.position.y - y;
	double n_x = path.poses.at(ind+1).pose.position.x - x;
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

bool Commande::fin()
{
	if(ind>path.poses.size()-1)
	{
		return true;
	}
	else
	{
		return false;
	}
}
