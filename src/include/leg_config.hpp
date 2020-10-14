#ifndef LEG_CONFIG_H
#define LEG_CONFIG_H

struct leg_config {
    double Ixx_hip3;
    double Ixy_hip3;
    double Ixz_hip3;

    double Iyx_hip3;
    double Iyy_hip3;
    double Iyz_hip3;

    double Izx_hip3;
    double Izy_hip3;
    double Izz_hip3;

    double Ixx_hip2;
    double Ixy_hip2;
    double Ixz_hip2;

    double Iyx_hip2;
    double Iyy_hip2;
    double Iyz_hip2;

    double Izx_hip2;
    double Izy_hip2;
    double Izz_hip2;


    double Ixx_hip1;
    double Ixy_hip1;
    double Ixz_hip1;

    double Iyx_hip1;
    double Iyy_hip1;
    double Iyz_hip1;

    double Izx_hip1;
    double Izy_hip1;
    double Izz_hip1;

    // "ul" is upper leg

    double Ixx_ul;
    double Ixy_ul;
    double Ixz_ul;

    double Iyx_ul;
    double Iyy_ul;
    double Iyz_ul;

    double Izx_ul;
    double Izy_ul;
    double Izz_ul;

    //"ll" is lower leg

    double Ixx_ll;
    double Ixy_ll;
    double Ixz_ll;

    double Iyx_ll;
    double Iyy_ll;
    double Iyz_ll;

    double Izx_ll;
    double Izy_ll;
    double Izz_ll;

    double Ixx_foot;
    double Ixy_foot;
    double Ixz_foot;

    double Iyx_foot;
    double Iyy_foot;
    double Iyz_foot;

    double Izx_foot;
    double Izy_foot;
    double Izz_foot;

    // Masses of each link in kg

    double m_hip3;
    double m_hip2;
    double m_hip1;
    double m_ul;
    double m_ll;
    double m_foot;

    // Distances from joint to joint (l1 is 0 because bas frame is at the same place as hip3 so there is only rotation) in m

    double l1_x;
    double l2_x;
    double l3_x;
    double l4_x;
    double l5_x;
    double l6_x;

    double l1_y;
    double l2_y;
    double l3_y;
    double l4_y;
    double l5_y;
    double l6_y;

    double l1_z;
    double l2_z;
    double l3_z;
    double l4_z;
    double l5_z;
    double l6_z;

    // Distances from joint n-1 to CoM of link n

    double l1_x_com;
    double l2_x_com;
    double l3_x_com;
    double l4_x_com;
    double l5_x_com;
    double l6_x_com;

    double l1_y_com;
    double l2_y_com;
    double l3_y_com;
    double l4_y_com;
    double l5_y_com;
    double l6_y_com;

    double l1_z_com;
    double l2_z_com;
    double l3_z_com;
    double l4_z_com;
    double l5_z_com;
    double l6_z_com;

    double g;

    double upper_torque_limit;
    double lower_torque_limit;
};

#endif