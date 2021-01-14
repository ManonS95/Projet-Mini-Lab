#ifndef COMMANDE_HPP
#define COMMANDE_HPP

#include <nav_msgs/Path.h>
#include <nav_msgs/Point.h>
#include "Pid.hpp"

class Commande {
	public:
		Commande(Path path);

		// Get
		double theta_error(Point current_pos);
		double distance(Point current_pos);

		// Methode
		void init(Path path);
		cmd_vel following(Point current_pos);

	private:
		Pid pid;
		Path path;
		Point checkpoint;
		double threshold;
		size_t ind;
};

#endif //COMMANDE_HPP
