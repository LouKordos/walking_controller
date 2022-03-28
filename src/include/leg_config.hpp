#ifndef LEG_CONFIG_H
#define LEG_CONFIG_H

struct leg_config {
    //Inertia tensors for each link around CoM
   
double Ixx_hip3 = 0.004505;
double Ixy_hip3 = 0;
double Ixz_hip3 = 0;

double Iyx_hip3 = 0;
double Iyy_hip3 = 0.004505;
double Iyz_hip3 = 0;

double Izx_hip3 = 0;
double Izy_hip3 = 0;
double Izz_hip3 = 0.008515;

double Ixx_hip2 = 0.004505;
double Ixy_hip2 = 0;
double Ixz_hip2 = 0;

double Iyx_hip2 = 0;
double Iyy_hip2 = 0.008515;
double Iyz_hip2 = 0;

double Izx_hip2 = 0;
double Izy_hip2 = 0;
double Izz_hip2 = 0.004505;


double Ixx_hip1 = 0.0017;
double Ixy_hip1 = 0;
double Ixz_hip1 = 0;

double Iyx_hip1 = 0;
double Iyy_hip1 = 0.022;
double Iyz_hip1 = 0;

double Izx_hip1 = 0;
double Izy_hip1 = 0;
double Izz_hip1 = 0.022;

// "ul" is upper leg

double Ixx_ul = 0.008673;
double Ixy_ul = 0;
double Ixz_ul = 0;

double Iyx_ul = 0;
double Iyy_ul = 0.008673;
double Iyz_ul = 0;

double Izx_ul = 0;
double Izy_ul = 0;
double Izz_ul = 2.436e-04;

//"ll" is lower leg

double Ixx_ll = 0.005;
double Ixy_ll = 0;
double Ixz_ll = 0;

double Iyx_ll = 0;
double Iyy_ll = 0.005;
double Iyz_ll = 0;

double Izx_ll = 0;
double Izy_ll = 0;
double Izz_ll = 2.078e-04;

double Ixx_foot = 2.733e-04;
double Ixy_foot = 0;
double Ixz_foot = 0;

double Iyx_foot = 0;
double Iyy_foot = 5.666e-05;
double Iyz_foot = 0;

double Izx_foot = 0;
double Izy_foot = 0;
double Izz_foot = 3.233e-04;

// Masses of each link in kg

double m_hip3 = 2.5;
double m_hip2 = 2.5;
double m_hip1 = 5;
double m_ul = 0.6;
double m_ll = 0.5;
double m_foot = 0.1;

// Distances from joint to joint (l1 is 0 because bas frame is at the same place as hip3 so there is only rotation) in m

double l1_x = 0;
double l2_x = 0;
double l3_x = 0;
double l4_x = 0;
double l5_x = 0;
double l6_x = 0;

double l1_y = 0;
double l2_y = -0.12;
double l3_y = 0.12;
double l4_y = 0;
double l5_y = 0;
double l6_y = 0;

double l1_z = 0;
double l2_z = 0.117;
double l3_z = 0;
double l4_z = 0.42;
double l5_z = 0.4;
double l6_z = 0.04;

// Distances from joint n-1 to CoM of link n in m

double l1_x_com = 0;
double l2_x_com = 0;
double l3_x_com = 0;
double l4_x_com = 0;
double l5_x_com = 0;
double l6_x_com = 0;

double l1_y_com = 0;
double l2_y_com = -0.12;
double l3_y_com = 0.12;
double l4_y_com = 0;
double l5_y_com = 0;
double l6_y_com = 0;

double l1_z_com = 0;
double l2_z_com = 0;
double l3_z_com = 0;
double l4_z_com = l4_z / 2 + 0.03716;
double l5_z_com = l5_z / 2;
double l6_z_com = l6_z - 0.01;

    // Gravity constant in m/s²

    double g = 9.81;

    double upper_torque_limit = 60;
    double lower_torque_limit = -60;
};

#endif