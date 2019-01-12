// EICbike.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Eigen/Dense>
#include <iostream>
#include "bike_plant.h"
#include"desired_trj.h"
#include"eiccontrol.h"
#include<fstream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector2d;
int main()
{
	double dt = 0.01;
	bike_plant mybike;
	VectorXd x(6), x_ini(6);
	//x,y,yaw,v,roll,droll
	x_ini << -0.5, -5.5, 0.0, 1.5, -0.1, 0.0;
	x = x_ini;
	desired_trj trj(6.0, 1.8); // radius, speed
	// initialize controller
	eiccontrol controller(mybike, trj);
	double angle_bem, acc, steer, dyaw_design;
	Vector2d dyaw_and_acc;
	VectorXd dxdt(6);
	int steps = 1900;
	MatrixXd x_collect(6, steps);
	MatrixXd desired_trj_collect(4, steps);
	MatrixXd bem_collect(1, steps);
	MatrixXd time_collect(1, steps);
	MatrixXd steer_collect(1, steps);
	for (int i = 0; i < steps; i++) {
		time_collect(i) = i*dt;
		x_collect.col(i) = x;
		desired_trj_collect(0, i) = trj.get_desired_xpos(i*dt);
		desired_trj_collect(1, i) = trj.get_desired_ypos(i*dt);
		desired_trj_collect(2, i) = trj.get_desired_dot_xpos(i*dt);
		desired_trj_collect(3, i) = trj.get_desired_dot_ypos(i*dt);
		dyaw_and_acc = controller.external_track_control(x, i*dt, 4.0, 1.8);
		dyaw_design = dyaw_and_acc(0), acc = dyaw_and_acc(1);
		angle_bem = controller.solveBEM(x, dyaw_design);
		bem_collect(i) = angle_bem;
		steer= controller.internal_stabilize_BEM(x, angle_bem, 150, 40);
		steer_collect(i) = steer;
		dxdt = mybike.simulate(x, steer, acc);
		x = x + dxdt*dt;
		//cout << x << endl;
	}
	
	ofstream output_file("bikesim.txt", ios::out | ios::trunc);
	output_file << x_collect << endl;
	output_file << desired_trj_collect << endl;
	output_file << bem_collect << endl;
	output_file << steer_collect << endl;
	output_file << time_collect;
	
	
    return 0;
}

