#ifndef COMMANDE_HPP
#define COMMANDE_HPP

#include <nav_msgs/Path.h>
#include <vector>
#include "Pid.hpp"

class Commande {
	public:
		Commande();
		Commande(Path path);

		// Get
		double theta_error(double theta);
		double distance(double x, double y);

		// Methode
		void init(Path path);
		double mot_command(double x, double y, double theta);
		std::vector<double> command_law(double x, double y, double theta);
		bool verification(double x, double y);
		bool fin();
	private:
		Pid pid;
		Path path;
		double threshold;
		size_t ind;
		double u1;
};

in
#endif //COMMANDE_HPP
