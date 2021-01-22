#ifndef COMMANDE_HPP
#define COMMANDE_HPP

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include "pid.hpp"

class Commande {
	public:
		Commande();
		Commande(nav_msgs::Path path);

		// Get
		double theta_error(double theta);
		double distance(double x, double y);

		// Methode
		void init(const nav_msgs::Path& path);
		double mot_command(double x, double y, double theta);
		std::vector<double> command_law(double x, double y, double theta);
		double K(double d, double theta_e);
		bool verification(double x, double y);
		bool fin();
	private:
		Pid pid;
		nav_msgs::Path path;
		double threshold;
		size_t ind;
		double u1;
};

#endif //COMMANDE_HPP
