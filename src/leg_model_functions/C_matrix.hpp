#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>
#include <iostream>

using Eigen::MatrixXd;

//Inertia tensors for each link around CoM

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

extern Eigen::Matrix<double, 5,1> C;

void update_C_left_leg(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double thetadot1, double thetadot2, double thetadot3, double thetadot4, double thetadot5);