
#ifndef USER_MAIN_OPERATION2_H_
#define USER_MAIN_OPERATION2_H_

#include "matrices_op2.h"

//Main program function

void desired_trajectory(double *v_r, double *w_r, double x_r, double y_r);
void velocity(matrix *v, double left_angular_velocity, double right_angular_velocity);
void error(double x, double y, double theta, double x_r, double y_r, double theta_r, double *e_x, double *e_y, double *e_theta);
void virtual_control(matrix *v_c, matrix *K, matrix *v_c_old, double e_x, double e_y, double e_theta, double v_r, double w_r);
void control_signal(matrix *u, matrix *v_c, matrix *v_c_old, matrix *v, matrix *K_4);
void torque(double theta, matrix *v, matrix *u, matrix *tau);
void voltage(double *voltage_left, double *voltage_right, double left_angular_velocity, double right_angular_velocity, matrix *tau);
void next_state(matrix *v, double *x, double *y, double *theta, double *x_r, double *y_r, double *theta_r, double w_r, double v_r);

#endif /* USER_MAIN_OPERATION2_H_ */
