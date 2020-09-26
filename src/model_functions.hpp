#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>

using Eigen::MatrixXd;

extern double Ixx_hip3;
extern double Ixy_hip3;
extern double Ixz_hip3;

extern double Iyx_hip3;
extern double Iyy_hip3;
extern double Iyz_hip3;

extern double Izx_hip3;
extern double Izy_hip3;
extern double Izz_hip3;

extern double Ixx_hip2;
extern double Ixy_hip2;
extern double Ixz_hip2;

extern double Iyx_hip2;
extern double Iyy_hip2;
extern double Iyz_hip2;

extern double Izx_hip2;
extern double Izy_hip2;
extern double Izz_hip2;


extern double Ixx_hip1;
extern double Ixy_hip1;
extern double Ixz_hip1;

extern double Iyx_hip1;
extern double Iyy_hip1;
extern double Iyz_hip1;

extern double Izx_hip1;
extern double Izy_hip1;
extern double Izz_hip1;

// "ul" is upper leg

extern double Ixx_ul;
extern double Ixy_ul;
extern double Ixz_ul;

extern double Iyx_ul;
extern double Iyy_ul;
extern double Iyz_ul;

extern double Izx_ul;
extern double Izy_ul;
extern double Izz_ul;

//"ll" is lower leg

extern double Ixx_ll;
extern double Ixy_ll;
extern double Ixz_ll;

extern double Iyx_ll;
extern double Iyy_ll;
extern double Iyz_ll;

extern double Izx_ll;
extern double Izy_ll;
extern double Izz_ll;

extern double Ixx_foot;
extern double Ixy_foot;
extern double Ixz_foot;

extern double Iyx_foot;
extern double Iyy_foot;
extern double Iyz_foot;

extern double Izx_foot;
extern double Izy_foot;
extern double Izz_foot;

// Masses of each link in kg

extern double m_hip3;
extern double m_hip2;
extern double m_hip1;
extern double m_ul;
extern double m_ll;
extern double m_foot;

// Distances from joint to joint (l1 is 0 because bas frame is at the same place as hip3 so there is only rotation) in m

extern double l1_x;
extern double l2_x;
extern double l3_x;
extern double l4_x;
extern double l5_x;
extern double l6_x;

extern double l1_y;
extern double l2_y;
extern double l3_y;
extern double l4_y;
extern double l5_y;
extern double l6_y;

extern double l1_z;
extern double l2_z;
extern double l3_z;
extern double l4_z;
extern double l5_z;
extern double l6_z;

// Distances from joint n-1 to CoM of link n

extern double l1_x_com;
extern double l2_x_com;
extern double l3_x_com;
extern double l4_x_com;
extern double l5_x_com;
extern double l6_x_com;

extern double l1_y_com;
extern double l2_y_com;
extern double l3_y_com;
extern double l4_y_com;
extern double l5_y_com;
extern double l6_y_com;

extern double l1_z_com;
extern double l2_z_com;
extern double l3_z_com;
extern double l4_z_com;
extern double l5_z_com;
extern double l6_z_com;

extern double g;

// Gravity constant in m/s²

extern double g;

extern double upper_torque_limit;
extern double lower_torque_limit;

extern Eigen::Matrix<double, 3, 3> h; // Damping ratio matrix
extern Eigen::Matrix<double, 3, 1> omega_desired; // Desired natural frequency of the leg

extern double Kp_orientation; // Position Gain for Euler angle orientation
extern double Kd_orientation; // Velocity Gain for Euler Angle orientation


extern std::mutex q_left_leg_mutex, q_dot_left_leg_mutex;

extern Eigen::Matrix<double, 5, 1> q_left_leg; // Leg angle vector / Model state
extern Eigen::Matrix<double, 5, 1> q_dot_left_leg; // Leg angular velocity vector / Differentiated model state
extern Eigen::Matrix<double, 5, 1> C_left_leg; // matrix containing the result of C * q_dot and the other terms based on the jupyter notebook
extern Eigen::Matrix<double, 5, 5> B_left_leg; // mass and inertia matrix of the leg model
extern Eigen::Matrix<double, 5, 1> G_left_leg; // gravity vector of the leg model. If directly applied as torques to each joint, it should compensate for gravity.
extern Eigen::Matrix<double, 3, 5> J_foot_left_leg; // Jacobian of the foot / end effector, also called the contact Jacobian.
extern Eigen::Matrix<double, 5, 5> J_foot_combined_left_leg; // Combined Jacobian with geometric positional part and analytical orientation part.
extern Eigen::Matrix<double, 3, 5> J_foot_dot_left_leg; // Time derivative of the contact / end effector Jacobian,
extern Eigen::Matrix<double, 3, 3> Lambda_left_leg; // "Desired Inertia matrix" of the leg, based on Jacobian and inertia matrix B / M
extern Eigen::Matrix<double, 5, 5> Kp_left_leg; // Cartesian Position gain matrix for calculation of torque setpoint
extern Eigen::Matrix<double, 5, 5> Kd_left_leg; // Derivative / Cartesian Velocity for calculation of torque setpoint
extern Eigen::Matrix<double, 5, 1> tau_ff_left_leg; // Vector containing feedforward torque based on Coriolis, Centrifugal, gravity and feed-forward acceleration terms.
extern Eigen::Matrix<double, 5, 1> tau_setpoint_left_leg; // Final torque setpoint calculated from above matrices and feedforward torque added.
extern Eigen::Matrix<double, 5, 1> foot_pos_left_leg; // Cartesian foot / end-effector position
extern Eigen::Matrix<double, 5, 1> foot_vel_left_leg; // Cartesian foot/ end-effector velocity
extern Eigen::Matrix<double, 5, 1> pos_desired_left_leg; // Desired cartesian foot / end-effector position + orientation (roll and yaw)
extern Eigen::Matrix<double, 5, 1> vel_desired_left_leg; // Desired cartesian foot / end-effector velocity + angular velocity
extern Eigen::Matrix<double, 3, 1> accel_desired_left_leg; // Desired cartesian foot / end-effector acceleration

// Euler Angle definitions:
// roll - around x - alpha - phi
// pitch - around y - beta - theta
// yaw - around z - gamma - psi

extern double phi_left_leg; // Roll, rotation around X, alpha
extern double theta_left_leg; // Pitch, rotation around Y, beta
extern double psi_left_leg; // Yaw, rotation arond Z, gamma

extern std::mutex q_right_leg_mutex, q_dot_right_leg_mutex;

extern Eigen::Matrix<double, 5, 1> q_right_leg; // Leg angle vector / Model state
extern Eigen::Matrix<double, 5, 1> q_dot_right_leg; // Leg angular velocity vector / Differentiated model state
extern Eigen::Matrix<double, 5, 1> C_right_leg; // matrix containing the result of C * q_dot and the other terms based on the jupyter notebook
extern Eigen::Matrix<double, 5, 5> B_right_leg; // mass and inertia matrix of the leg model
extern Eigen::Matrix<double, 5, 1> G_right_leg; // gravity vector of the leg model. If directly applied as torques to each joint, it should compensate for gravity.
extern Eigen::Matrix<double, 3, 5> J_foot_right_leg; // Jacobian of the foot / end effector, also called the contact Jacobian.
extern Eigen::Matrix<double, 5, 5> J_foot_combined_right_leg; // Combined Jacobian with geometric positional part and analytical orientation part.
extern Eigen::Matrix<double, 3, 5> J_foot_dot_right_leg; // Time derivative of the contact / end effector Jacobian,
extern Eigen::Matrix<double, 3, 3> Lambda_right_leg; // "Desired Inertia matrix" of the leg, based on Jacobian and inertia matrix B / M
extern Eigen::Matrix<double, 5, 5> Kp_right_leg; // Cartesian Position gain matrix for calculation of torque setpoint
extern Eigen::Matrix<double, 5, 5> Kd_right_leg; // Derivative / Cartesian Velocity for calculation of torque setpoint
extern Eigen::Matrix<double, 5, 1> tau_ff_right_leg; // Vector containing feedforward torque based on Coriolis, Centrifugal, gravity and feed-forward acceleration terms.
extern Eigen::Matrix<double, 5, 1> tau_setpoint_right_leg; // Final torque setpoint calculated from above matrices and feedforward torque added.
extern Eigen::Matrix<double, 5, 1> foot_pos_right_leg; // Cartesian foot / end-effector position
extern Eigen::Matrix<double, 5, 1> foot_vel_right_leg; // Cartesian foot/ end-effector velocity
extern Eigen::Matrix<double, 5, 1> pos_desired_right_leg; // Desired cartesian foot / end-effector position + orientation (roll and yaw)
extern Eigen::Matrix<double, 5, 1> vel_desired_right_leg; // Desired cartesian foot / end-effector velocity + angular velocity
extern Eigen::Matrix<double, 3, 1> accel_desired_right_leg; // Desired cartesian foot / end-effector acceleration

// Euler Angle definitions:
// roll - around x - alpha - phi
// pitch - around y - beta - theta
// yaw - around z - gamma - psi

extern double phi_right_leg; // Roll, rotation around X, alpha
extern double theta_right_leg; // Pitch, rotation around Y, beta
extern double psi_right_leg; // Yaw, rotation arond Z, gamma

void update_q_left_leg(double theta1, double theta2, double theta3, double theta4, double theta5);

void update_q_dot_left_leg(double theta1dot, double theta2dot, double theta3dot, double theta4dot, double theta5dot);

void update_B_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double theta2dot, double theta3dot, double theta4dot, double theta5dot);

void update_C_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5);

void update_G_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

void update_J_foot_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

void update_J_foot_combined_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5);

void update_J_foot_dot_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5);

void update_Lambda_left_leg();

void update_orientation_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

void update_Kp_left_leg();

void update_Kd_left_leg();

void update_tau_ff_left_leg(Eigen::Matrix<double, 5, 1> q_dot);

void update_foot_pos_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

void update_foot_vel_left_leg(Eigen::Matrix<double, 5, 1> q_dot);

void update_tau_setpoint_left_leg();

void update_q_right_leg(double theta1, double theta2, double theta3, double theta4, double theta5);

void update_q_dot_right_leg(double theta1dot, double theta2dot, double theta3dot, double theta4dot, double theta5dot);

void update_B_right_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double theta2dot, double theta3dot, double theta4dot, double theta5dot);

void update_C_right_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5);

void update_G_right_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

void update_J_foot_right_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

void update_J_foot_combined_right_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5);

void update_J_foot_dot_right_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5);

void update_Lambda_right_leg();

void update_orientation_right_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

void update_Kp_right_leg();

void update_Kd_right_leg();

void update_tau_ff_right_leg(Eigen::Matrix<double, 5, 1> q_dot);

void update_foot_pos_right_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5);

void update_foot_vel_right_leg(Eigen::Matrix<double, 5, 1> q_dot);

void update_tau_setpoint_right_leg();