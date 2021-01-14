#include "pid.hpp"



Pid::Pid(double Kp, double Ki, double Kd, double saturation) : Kp(Kp), Ki(Ki), Kd(Kd), saturation(saturation)
{}

double Pid::correcteur(double erreur)
{
	
}