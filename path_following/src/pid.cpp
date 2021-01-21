#include "pid.hpp"
#include <cmath>


Pid::Pid(double Kp, double Ki, double Kd, double saturation, double dt) : Kp(Kp), Ki(Ki), Kd(Kd), saturation(saturation), dt(dt)
{
	pre_e = 0;
	sum_e = 0;	
}

double Pid::correcteur(double erreur)
{
	sum_e += (erreur+pre_e)*dt/2;
	if(sum_e>saturation)
	{
		sum_e = saturation;
	}
	else if(sum_e<-saturation){
		sum_e = -saturation;
	}
	double de = (erreur-pre_e)/dt;
	double gain = Kp*erreur + Ki*sum_e + Kd * de;
	pre_e = erreur;
	return gain;
}
