#ifndef MODEL_FUNCTIONS_H
#define MODEL_FUNCTIONS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>
#include <iostream>
using Eigen::MatrixXd;

#include "leg_config.hpp"

void update_q(double theta1, double theta2, double theta3, double theta4, double theta5, Eigen::Matrix<double, 5, 1> &q, leg_config &config);

void update_q_dot(double theta1dot, double theta2dot, double theta3dot, double theta4dot, double theta5dot,  Eigen::Matrix<double, 5, 1> &q_dot, leg_config &config);

void update_B(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double theta2dot, double theta3dot, double theta4dot, double theta5dot, Eigen::Matrix<double, 5, 5> &B, leg_config &config);

void update_C(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5, Eigen::Matrix<double, 5, 1> &C, leg_config &config);

void update_G(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, Eigen::Matrix<double, 5, 1> &G, leg_config &config);

void update_J_foot(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, Eigen::Matrix<double, 3, 5> &J_foot, leg_config &config);

void update_J_foot_combined(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5, Eigen::Matrix<double, 5, 5> &J_foot_combined, leg_config &config);

void update_J_foot_dot(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5, Eigen::Matrix<double, 3, 5> &J_foot_dot, leg_config &config);

void update_Lambda(Eigen::Matrix<double, 3, 5> &J_foot, Eigen::Matrix<double, 5, 5> &B, Eigen::Matrix<double, 3, 3> &Lambda);

void update_orientation(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double &phi, double &theta, double &psi);

void update_Kp(Eigen::Matrix<double, 3,3> &Lambda, Eigen::Matrix<double, 3, 1> omega_desired, double &Kp_orientation, Eigen::Matrix<double, 5, 5> &Kp);

void update_Kd(Eigen::Matrix<double, 3,3> &Lambda, Eigen::Matrix<double, 3, 1> omega_desired, Eigen::Matrix<double, 3, 3> &h, double &Kd_orientation, Eigen::Matrix<double, 5, 5> &Kd);

void update_tau_ff(Eigen::Matrix<double, 5, 1> &G, Eigen::Matrix<double, 5, 1> &C, Eigen::Matrix<double, 3, 5> &J_foot, Eigen::Matrix<double, 3, 5> &J_foot_dot, Eigen::Matrix<double, 3, 3> &Lambda, Eigen::Matrix<double, 3, 1> &accel_desired, Eigen::Matrix<double, 5, 1> &q_dot, Eigen::Matrix<double, 5, 1> &tau_ff);

void update_foot_pos(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double &phi, double &psi, Eigen::Matrix<double, 5, 1> &foot_pos, leg_config &config);

void update_foot_vel(Eigen::Matrix<double, 5, 5> &J_foot_combined, Eigen::Matrix<double, 5, 1> &q_dot, Eigen::Matrix<double, 5, 1> &foot_vel);

void update_tau_setpoint(Eigen::Matrix<double, 5, 5> &J_foot_combined, Eigen::Matrix<double, 5, 5> &Kp, Eigen::Matrix<double, 5, 1> &pos_desired, Eigen::Matrix<double, 5, 1> &foot_pos, Eigen::Matrix<double, 5, 5> &Kd, Eigen::Matrix<double, 5, 1> &vel_desired, Eigen::Matrix<double, 5, 1> &foot_vel, Eigen::Matrix<double, 5, 1> &tau_ff, Eigen::Matrix<double, 5, 1> &tau_setpoint);

#endif