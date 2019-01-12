#ifndef EICCONTROL_H
#define EICCONTROL_H


#include "desired_trj.h"
#include "bike_plant.h"

using Eigen::Vector2d;

class eiccontrol
{
public:
	eiccontrol(bike_plant, desired_trj);
	~eiccontrol();
	Vector2d external_track_control(VectorXd, double, double, double);
	double solveBEM(VectorXd, double);
	double internal_stabilize_BEM(VectorXd, double, double, double);
private:
	desired_trj trj2follow;
	bike_plant  model;
};

eiccontrol::eiccontrol(bike_plant md, desired_trj trj):model(md),trj2follow(trj)
{
}

eiccontrol::~eiccontrol()
{
}

Vector2d eiccontrol::external_track_control(VectorXd x, double t, double kp, double kd) {
	double xpos_des = trj2follow.get_desired_xpos(t);
	double ypos_des = trj2follow.get_desired_ypos(t);
	double xvel_des = trj2follow.get_desired_dot_xpos(t);
	double yvel_des = trj2follow.get_desired_dot_ypos(t);
	double xacc_des = trj2follow.get_desired_ddot_xpos(t);
	double yacc_des = trj2follow.get_desired_ddot_ypos(t);

	double xpos = x(0), ypos = x(1), yaw = x(2), speed = x(3);
	double xvel = speed*cos(yaw), yvel = speed*sin(yaw);
	double ddx_design = xacc_des - kp*(xpos - xpos_des) - kd*(xvel - xvel_des);
	double ddy_design = yacc_des - kp*(ypos - ypos_des) - kd*(yvel - yvel_des);
	double d_yaw_design = (-sin(yaw)*ddx_design + cos(yaw)*ddy_design) / speed;
	double acc_design = cos(yaw)*ddx_design + sin(yaw)*ddy_design;
	Vector2d Uext(d_yaw_design, acc_design);
	return Uext;
}

double eiccontrol::solveBEM(VectorXd x, double d_yaw_design) {
	double bem = model.balance_equilibrium_manifold(x, d_yaw_design);
	return bem;
}

double eiccontrol::internal_stabilize_BEM(VectorXd x, double bem, double kp, double kd) {
	double roll = x(4);// roll angle x(4)
	double droll = x(5);// roll velocity x(5)
	double ddroll_design = -kp*(roll - bem) - kd*(droll - 0);
	// inverse dynamics controller
	// ddroll =f+g*tan(steer)
	double steer = atan((ddroll_design - model.get_model_f(x)) / model.get_model_g(x));
	return steer;
}

#endif // !EICCONTROL_H
