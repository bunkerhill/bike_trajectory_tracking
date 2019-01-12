#ifndef BIKE_PLANT_H
#define BIKE_PLANT_H

#include <Eigen/Dense>
using Eigen::VectorXd;


class bike_plant
{
public:
	bike_plant();
	~bike_plant();
	VectorXd simulate(VectorXd, double, double);
	double balance_equilibrium_manifold(VectorXd, double);

	// ddroll=f+g*steer
	double get_model_g(VectorXd);
	double get_model_f(VectorXd);


private:

	double mb;
	double Jb;
	double hb;
	double lb;
	double l ;
	double xi;
	double lt;
	double	g;


};

bike_plant::bike_plant()
{
	mb = 51;
	Jb = 2.5;
	hb = 0.64;
	lb = 0.27;
	l = 1.1;
	xi = 20.0 / 180.0*3.14159;
	lt = 0.06;
	g = 9.8;
}

bike_plant::~bike_plant()
{
}

VectorXd bike_plant::simulate(VectorXd x, double steer, double acc) {
	double xpos = x(0); // X position x(0)
	double ypos = x(1);// Y position x(1)
	double yaw = x(2);// Yaw angle  x(2)
	double speed = x(3); // rear wheel speed x(3)
	double roll = x(4);// roll angle x(4)
	double droll = x(5);// roll velocity x(5)

	// external dynamics
	double dxpos = speed*cos(yaw);
	double dypos = speed*sin(yaw);
	double dyaw = speed*tan(steer)*cos(xi) / l / cos(roll);
	// internal dynamics
	double ddroll = (g*sin(roll) - cos(roll)*speed*dyaw)*mb*hb /(mb*hb*hb+Jb);

	VectorXd dx(6);
	dx << dxpos, dypos, dyaw, acc, droll, ddroll;
	return dx;
}

double bike_plant::balance_equilibrium_manifold(VectorXd x,double d_yaw_design) {
	double speed = x(3);
	double bem = speed*d_yaw_design / g;
	return bem;
}

double bike_plant::get_model_f(VectorXd x) {
	double roll = x(4);// roll angle x(4)
	double  f = mb*hb*g*sin(roll) / (mb*hb*hb + Jb);
	return f;
}

double bike_plant::get_model_g(VectorXd x) {
	double speed = x(3);
	double g = -mb*hb*speed*speed*cos(xi) / l / (mb*hb*hb + Jb);
	return g;
}







#endif // !BIKE_PLANT_H
