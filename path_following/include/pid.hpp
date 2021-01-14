#ifndef PID_HPP
#define PID_HPP

class Pid{
	public:
		Pid(double Kp, double Ki, double Kd, double saturation);

		double correcteur(double erreur);
	private:
		double Kp;
		double Ki;
		double Kd;
		double saturation;
		double pre_e;
		double sum_e;
};

#endif //PID_HPP
