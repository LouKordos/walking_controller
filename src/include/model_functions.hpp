#ifndef MODEL_FUNCTIONS_H
#define MODEL_FUNCTIONS_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>
#include <iostream>
using Eigen::MatrixXd;

#include "leg_config.hpp"

void update_q(double theta2, double theta3, double theta4, double theta5, Eigen::Matrix<double, 4, 1> &q);

void update_q_dot(double theta2dot, double theta3dot, double theta4dot, double theta5dot,  Eigen::Matrix<double, 4, 1> &q_dot);

void update_B(double theta_2, double theta_3, double theta_4, double theta_5, double theta2dot, double theta3dot, double theta4dot, double theta5dot, Eigen::Matrix<double, 4, 4> &B, leg_config &config);

void update_C(double theta_2, double theta_3, double theta_4, double theta_5, double thetadot2, double thetadot3, double thetadot4, double thetadot5, Eigen::Matrix<double, 4, 1> &C, leg_config &config);

void update_G(double theta_2, double theta_3, double theta_4, double theta_5, Eigen::Matrix<double, 4, 1> &G, leg_config &config);

void update_J_foot(double theta_2, double theta_3, double theta_4, double theta_5, Eigen::Matrix<double, 3, 4> &J_foot, leg_config &config);

void update_J_foot_combined(double theta_2, double theta_3, double theta_4, double theta_5, double thetadot2, double thetadot3, double thetadot4, double thetadot5, Eigen::Matrix<double, 4, 4> &J_foot_combined, leg_config &config);

void update_J_foot_dot(double theta_2, double theta_3, double theta_4, double theta_5, double thetadot2, double thetadot3, double thetadot4, double thetadot5, Eigen::Matrix<double, 3, 4> &J_foot_dot, leg_config &config);

void update_Lambda(Eigen::Matrix<double, 3, 4> &J_foot, Eigen::Matrix<double, 4, 4> &B, Eigen::Matrix<double, 3, 3> &Lambda);

void update_orientation(double theta_2, double theta_3, double theta_4, double theta_5, double &phi, double &theta);

void update_Kp(Eigen::Matrix<double, 3,3> &Lambda, Eigen::Matrix<double, 3, 1> omega_desired, double &Kp_orientation, Eigen::Matrix<double, 4, 4> &Kp);

void update_Kd(Eigen::Matrix<double, 3,3> &Lambda, Eigen::Matrix<double, 3, 1> omega_desired, Eigen::Matrix<double, 3, 3> &h, double &Kd_orientation, Eigen::Matrix<double, 4, 4> &Kd);

void update_tau_ff(Eigen::Matrix<double, 4, 1> &G, Eigen::Matrix<double, 4, 1> &C, Eigen::Matrix<double, 3, 4> &J_foot, Eigen::Matrix<double, 3, 4> &J_foot_dot, Eigen::Matrix<double, 3, 3> &Lambda, Eigen::Matrix<double, 3, 1> &accel_desired, Eigen::Matrix<double, 4, 1> &q_dot, Eigen::Matrix<double, 4, 1> &tau_ff);

void update_foot_pos(double theta_2, double theta_3, double theta_4, double theta_5, double &phi, Eigen::Matrix<double, 4, 1> &foot_pos, leg_config &config);

void update_foot_vel(Eigen::Matrix<double, 4, 4> &J_foot_combined, Eigen::Matrix<double, 4, 1> &q_dot, Eigen::Matrix<double, 4, 1> &foot_vel);

void update_tau_setpoint(Eigen::Matrix<double, 4, 4> &J_foot_combined, Eigen::Matrix<double, 4, 4> &Kp, Eigen::Matrix<double, 4, 1> &pos_desired, Eigen::Matrix<double, 4, 1> &foot_pos, Eigen::Matrix<double, 4, 4> &Kd, Eigen::Matrix<double, 4, 1> &vel_desired, Eigen::Matrix<double, 4, 1> &foot_vel, Eigen::Matrix<double, 4, 1> &tau_ff, Eigen::Matrix<double, 4, 1> &tau_setpoint);

#endif