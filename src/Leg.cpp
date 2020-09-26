

class Leg {
    public: Leg() {

    }

    public: theta1;
    public: theta2;
    public: theta3;
    public: theta4;
    public: theta5;

    #pragma region Constants

    //Inertia tensors for each link around CoM

    public: static const double Ixx_hip3 = 1.249e-04;
    public: static const double Ixy_hip3 = 0;
    public: static const double Ixz_hip3 = 0;

    public: static const double Iyx_hip3 = 0;
    public: static const double Iyy_hip3 = 9.250e-05;
    public: static const double Iyz_hip3 = 0;

    public: static const double Izx_hip3 = 0;
    public: static const double Izy_hip3 = 0;
    public: static const double Izz_hip3 = 9.250e-05;

    public: static const double Ixx_hip2 = 1.249e-04;
    public: static const double Ixy_hip2 = 0;
    public: static const double Ixz_hip2 = 0;

    public: static const double Iyx_hip2 = 0;
    public: static const double Iyy_hip2 = 9.250e-05;
    public: static const double Iyz_hip2 = 0;

    public: static const double Izx_hip2 = 0;
    public: static const double Izy_hip2 = 0;
    public: static const double Izz_hip2 = 9.250e-05;


    public: static const double Ixx_hip1 = 1.249e-04;
    public: static const double Ixy_hip1 = 0;
    public: static const double Ixz_hip1 = 0;

    public: static const double Iyx_hip1 = 0;
    public: static const double Iyy_hip1 = 9.250e-05;
    public: static const double Iyz_hip1 = 0;

    public: static const double Izx_hip1 = 0;
    public: static const double Izy_hip1 = 0;
    public: static const double Izz_hip1 = 9.250e-05;

    // "ul" is upper leg

    public: static const double Ixx_ul = 8.5e-04;
    public: static const double Ixy_ul = 0;
    public: static const double Ixz_ul = 0;

    public: static const double Iyx_ul = 0;
    public: static const double Iyy_ul = 8.5e-04;
    public: static const double Iyz_ul = 0;

    public: static const double Izx_ul = 0;
    public: static const double Izy_ul = 0;
    public: static const double Izz_ul = 1e-04;

    //"ll" is lower leg

    public: static const double Ixx_ll = 7.982e-04;
    public: static const double Ixy_ll = 0;
    public: static const double Ixz_ll = 0;

    public: static const double Iyx_ll = 0;
    public: static const double Iyy_ll = 7.982e-04;
    public: static const double Iyz_ll = 0;

    public: static const double Izx_ll = 0;
    public: static const double Izy_ll = 0;
    public: static const double Izz_ll = 9.378e-05;

    public: static const double Ixx_foot = 1.42e-04;
    public: static const double Ixy_foot = 0;
    public: static const double Ixz_foot = 0;

    public: static const double Iyx_foot = 0;
    public: static const double Iyy_foot = 2.8e-05;
    public: static const double Iyz_foot = 0;

    public: static const double Izx_foot = 0;
    public: static const double Izy_foot = 0;
    public: static const double Izz_foot = 1.6e-04;

    // Masses of each link in kg

    public: static const double m_hip3 = 0.1;
    public: static const double m_hip2 = 0.1;
    public: static const double m_hip1 = 0.1;
    public: static const double m_ul = 0.11;
    public: static const double m_ll = 0.1;
    public: static const double m_foot = 0.05;

    // Distances from joint to joint (l1 is 0 because bas frame is at the same place as hip3 so there is only rotation) in m

    public: static const double l1_x = 0;
    public: static const double l2_x = 0;
    public: static const double l3_x = 0;
    public: static const double l4_x = 0;
    public: static const double l5_x = 0;
    public: static const double l6_x = 0;

    public: static const double l1_y = 0;
    public: static const double l2_y = 0;
    public: static const double l3_y = 0;
    public: static const double l4_y = 0;
    public: static const double l5_y = 0;
    public: static const double l6_y = 0;

    public: static const double l1_z = 0;
    public: static const double l2_z = 0.145;
    public: static const double l3_z = 0.145;
    public: static const double l4_z = 0.41;
    public: static const double l5_z = 0.375;
    public: static const double l6_z = 0.04;

    // Distances from joint n-1 to CoM of link n

    public: static const double l1_x_com = 0;
    public: static const double l2_x_com = 0;
    public: static const double l3_x_com = 0;
    public: static const double l4_x_com = 0;
    public: static const double l5_x_com = 0;
    public: static const double l6_x_com = 0;

    public: static const double l1_y_com = 0;
    public: static const double l2_y_com = 0;
    public: static const double l3_y_com = 0;
    public: static const double l4_y_com = 0;
    public: static const double l5_y_com = 0;
    public: static const double l6_y_com = 0;

    public: static const double l1_z_com = 0;
    public: static const double l2_z_com = 0.145;
    public: static const double l3_z_com = 0;
    public: static const double l4_z_com = l4_z / 2 + 0.03716;
    public: static const double l5_z_com = l5_z / 2;
    public: static const double l6_z_com = l6_z - 0.01;
    
    public: void update() {

    }
}