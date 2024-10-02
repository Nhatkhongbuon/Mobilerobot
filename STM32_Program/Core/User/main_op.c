#include "matrices_op2.h"

// Robot parameters
extern double sampling_interval;
extern double m, d, I, r, R;
extern double t1, t2;

/*
(Global variable)
double sampling_interval;
double m, d, I, r, R;

(Delclare in main)
double x, y, theta, x_r, y_r, theta_r;
double e_x, e_y, e_theta;
double w_r, v_r;
double k_4;
matrix K_4;
matrix K;
matrix u;
matrix v_c;
matrix v_c_old;
matrix tau;
*/

// Motor parameter
extern double k_phi;
extern double R_a;

void error(double x, double y, double theta, double x_r, double y_r, double theta_r, double *e_x, double *e_y, double *e_theta)
{
    *e_x = cos(theta) * (x_r - x) + sin(theta) * (y_r - y);
    *e_y = (-1) * sin(theta) * (x_r - x) + cos(theta) * (y_r - y);
    *e_theta = theta_r - theta;
}

// Caculate virtural control signal
// Must allocate v_c matrix and K matrix in the main program
void virtual_control(matrix *v_c, matrix *K, matrix *v_c_old, double e_x, double e_y, double e_theta, double v_r, double w_r)
{
    // Store the previous value of v_c
    v_c_old->index[0][0] = v_c->index[0][0];
    v_c_old->index[1][0] = v_c->index[1][0];

    // Calculate new v_c
    v_c->index[0][0] = v_r * cos(e_theta) + K->index[0][0] * e_x;
    v_c->index[1][0] = w_r + K->index[1][0] * v_r * e_y + K->index[2][0] * v_r * sin(e_theta);
}

// Calculate the control signal u
// Must allocate u matrix in the main program
void control_signal(matrix *u, matrix *v_c, matrix *v_c_old, matrix *v, matrix *K_4)
{

    double x  = (v_c->index[0][0] - v_c_old->index[0][0]) / sampling_interval;
    double y  = (v_c->index[1][0] - v_c_old->index[1][0]) / sampling_interval;

    subtraction(v_c, v);
    matrix C;
    mutiplication(K_4, v_c, &C);
    u->index[0][0] = x + C.index[0][0];
    u->index[1][0] = y + C.index[1][0];
    
    deallocate_matrix(&C);
}

// Calculate future coordinates of the robot
// Call after applying voltages to motors
void next_state(matrix *v, double *x, double *y, double *theta, double *x_r, double *y_r, double *theta_r, double w_r, double v_r)
{
    double derivative_x = cos(*theta) * v->index[0][0];
    double derivative_y = sin(*theta) * v->index[0][0];
    double derivative_theta = v->index[1][0];

    // Calculate future coordinates
    *x = *x + sampling_interval * derivative_x;
    *y = *y + sampling_interval * derivative_y;
    *theta = *theta + sampling_interval * derivative_theta;

    double derivative_x_r = cos(*theta_r) * v_r;
    double derivative_y_r = sin(*theta_r) * v_r;
    double derivative_theta_r = w_r;

    // Calculate future reference coordinates
    *x_r = *x_r + sampling_interval * derivative_x_r;
    *y_r = *y_r + sampling_interval * derivative_y_r;
    *theta_r = *theta_r + sampling_interval * derivative_theta_r;
}

// Calculate torque signal
// Must allocate tau matrix in the main program
void torque(double theta, matrix *v, matrix *u, matrix *tau)
{
double derivative_theta = v->index[1][0];
matrix M;
allocate_matrix(&M, 2, 2);
M.index[0][0] = m;
M.index[1][1] = I;

matrix V;
allocate_matrix(&V, 2, 2);
V.index[0][1] = m * d * derivative_theta;
V.index[1][0] = - m * d * derivative_theta;

matrix B;
allocate_matrix(&B, 2, 2);
B.index[0][0] = B.index[0][1] = 1 / r;
B.index[1][0] = R / r;
B.index[1][1] = - R / r;

matrix inv_B;
inverse(&B, &inv_B);
deallocate_matrix(&B);

matrix M_u;
mutiplication(&M, u, &M_u);
deallocate_matrix(&M);

matrix V_v;
mutiplication(&V, v, &V_v);
deallocate_matrix(&V);

addition(&M_u, &V_v);
deallocate_matrix(&V_v);

matrix torque;
mutiplication(&inv_B, &M_u, &torque);
deallocate_matrix(&inv_B);
deallocate_matrix(&M_u);


tau->index[0][0] = torque.index[0][0];
tau->index[1][0] = torque.index[1][0];
t1 = torque.index[0][0];
t2 = torque.index[1][0];

deallocate_matrix(&torque);
}

// Calculate the linear velocity and angular velocity of the vehicle
void velocity(matrix *v, double left_angular_velocity, double right_angular_velocity)
{
    v->index[0][0] = r / 2 * (left_angular_velocity + right_angular_velocity);
    v->index[1][0] = r / (2 * R) * (right_angular_velocity - left_angular_velocity);
}

 //Calculate voltage for the motor
void voltage(double *voltage_left, double *voltage_right, double left_angular_velocity, double right_angular_velocity, matrix *tau)
{

    *voltage_left = k_phi * left_angular_velocity * 30 + R_a * tau->index[1][0] / k_phi;
    *voltage_right = k_phi * right_angular_velocity * 30 + R_a * tau->index[0][0] / k_phi;
}

//void voltage(double *voltage_left, double *voltage_right, double left_angular_velocity, double right_angular_velocity, matrix *tau)
//{
//    *voltage_left =  R_a * tau->index[1][0] / k_phi;
//    *voltage_right =  R_a * tau->index[0][0] / k_phi;
//}

//void voltage(double *voltage_left, double *voltage_right, double left_angular_velocity, double right_angular_velocity, matrix *tau)
//{
//    *voltage_left = k_phi * left_angular_velocity * 30 + 1;
//    *voltage_right = k_phi * right_angular_velocity * 30 + 1;
//}

