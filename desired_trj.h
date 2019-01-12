#ifndef DESIRED_TRJ_H
#define DESIRED_TRJ_H

#include <Eigen/Dense>
class desired_trj
{
public:
	desired_trj(double,double);
	~desired_trj();
	double get_desired_xpos(double);
	double get_desired_dot_xpos(double);
	double get_desired_ddot_xpos(double);

	double get_desired_ypos(double);
	double get_desired_dot_ypos(double);
	double get_desired_ddot_ypos(double);

private:
	double R; // radius of the circle
	double v; // tangent velocity of the circle

};


double desired_trj::get_desired_xpos(double t) {
	return R*cos(v / R*t - 3.14159 / 2.0);
}

double desired_trj::get_desired_ypos(double t) {
	return R*sin(v / R*t - 3.14159 / 2.0);
}

double desired_trj::get_desired_dot_xpos(double t) {
	return -v*sin(v / R*t - 3.14159 / 2.0);
}
double desired_trj::get_desired_dot_ypos(double t) {
	return v*cos(v / R*t - 3.14159 / 2.0);
}

double desired_trj::get_desired_ddot_xpos(double t) {
	return -v*v/R*cos(v / R*t - 3.14159 / 2.0);
}
double desired_trj::get_desired_ddot_ypos(double t) {
	return -v*v/R*sin(v / R*t - 3.14159 / 2.0);
}




desired_trj::desired_trj(double radius, double speed) :R(radius), v(speed)
{
}

desired_trj::~desired_trj()
{
}















#endif // !DESIRED_TRJ_H
