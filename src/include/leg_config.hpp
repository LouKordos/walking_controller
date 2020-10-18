#ifndef LEG_CONFIG_H
#define LEG_CONFIG_H

struct leg_config {
    //Inertia tensors for each link around CoM
    double Ixx_hip3 = 1.249e-04 * 10;
    double Ixy_hip3 = 0;
    double Ixz_hip3 = 0;

    double Iyx_hip3 = 0;
    double Iyy_hip3 = 9.250e-05 * 10;
    double Iyz_hip3 = 0;

    double Izx_hip3 = 0;
    double Izy_hip3 = 0;
    double Izz_hip3 = 9.250e-05 * 10;

    double Ixx_hip2 = 1.249e-04 * 10;
    double Ixy_hip2 = 0;
    double Ixz_hip2 = 0;

    double Iyx_hip2 = 0;
    double Iyy_hip2 = 9.250e-05 * 10;
    double Iyz_hip2 = 0;

    double Izx_hip2 = 0;
    double Izy_hip2 = 0;
    double Izz_hip2 = 9.250e-05 * 10;


    double Ixx_hip1 = 1.249e-04 * 10;
    double Ixy_hip1 = 0;
    double Ixz_hip1 = 0;

    double Iyx_hip1 = 0;
    double Iyy_hip1 = 9.250e-05 * 10;
    double Iyz_hip1 = 0;

    double Izx_hip1 = 0;
    double Izy_hip1 = 0;
    double Izz_hip1 = 9.250e-05 * 10;

    // "ul" is upper leg

    double Ixx_ul = 8.5e-04;
    double Ixy_ul = 0;
    double Ixz_ul = 0;

    double Iyx_ul = 0;
    double Iyy_ul = 8.5e-04;
    double Iyz_ul = 0;

    double Izx_ul = 0;
    double Izy_ul = 0;
    double Izz_ul = 1e-04;

    //"ll" is lower leg

    double Ixx_ll = 7.982e-04;
    double Ixy_ll = 0;
    double Ixz_ll = 0;

    double Iyx_ll = 0;
    double Iyy_ll = 7.982e-04;
    double Iyz_ll = 0;

    double Izx_ll = 0;
    double Izy_ll = 0;
    double Izz_ll = 9.378e-05;

    double Ixx_foot = 1.42e-04;
    double Ixy_foot = 0;
    double Ixz_foot = 0;

    double Iyx_foot = 0;
    double Iyy_foot = 2.8e-05;
    double Iyz_foot = 0;

    double Izx_foot = 0;
    double Izy_foot = 0;
    double Izz_foot = 1.6e-04;

    // Masses of each link in kg

    double m_hip3 = 1;
    double m_hip2 = 1;
    double m_hip1 = 1;
    double m_ul = 0.11;
    double m_ll = 0.1;
    double m_foot = 0.05;

    // Distances from joint to joint (l1 is 0 because bas frame is at the same place as hip3 so there is only rotation) in m

    double l1_x = 0;
    double l2_x = 0;
    double l3_x = 0;
    double l4_x = 0;
    double l5_x = 0;
    double l6_x = 0;

    double l1_y = 0;
    double l2_y = 0;
    double l3_y = 0;
    double l4_y = 0;
    double l5_y = 0;
    double l6_y = 0;

    double l1_z = 0;
    double l2_z = 0.145;
    double l3_z = 0.145;
    double l4_z = 0.41;
    double l5_z = 0.375;
    double l6_z = 0.04;

    // Distances from joint n-1 to CoM of link n

    double l1_x_com = 0;
    double l2_x_com = 0;
    double l3_x_com = 0;
    double l4_x_com = 0;
    double l5_x_com = 0;
    double l6_x_com = 0;

    double l1_y_com = 0;
    double l2_y_com = 0;
    double l3_y_com = 0;
    double l4_y_com = 0;
    double l5_y_com = 0;
    double l6_y_com = 0;

    double l1_z_com = 0;
    double l2_z_com = 0.145;
    double l3_z_com = 0.145;
    double l4_z_com = l4_z / 2 + 0.03716;
    double l5_z_com = l5_z / 2;
    double l6_z_com = l6_z - 0.01;

    // Gravity constant in m/s²

    double g = 9.81;

    double upper_torque_limit = 60;
    double lower_torque_limit = -60;
};

#endif