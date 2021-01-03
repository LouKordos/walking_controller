#include "include/Helpers.hpp"

// Helper function for constraining a double precision float to limits and filtering out "nan" and "inf"
void constrain(double &value, double lower_limit, double upper_limit) {
    if(isnan(value) || isinf(value)) {
        value = 0;
    }
    else if(value > upper_limit) {
        value = upper_limit;
    }
    else if(value < lower_limit) {
        value = lower_limit;
    }
}

void constrain_int(int &value, int lower_limit, int upper_limit) {
    if(isnan(value) || isinf(value)) {
        value = 0;
    }
    else if(value > upper_limit) {
        value = upper_limit;
    }
    else if(value < lower_limit) {
        value = lower_limit;
    }
}

unsigned long long factorial(long n) {
    unsigned long long temp = 1;

    for(int i = 1; i <=n; ++i)
    {
        temp *= i;
    }

    return temp;
}

//TODO: Make trajectory matrix length dynamic, it is currently 1 second long, assuming 1ms time steps
// First three columns are position in XYZ, last three columns are velocities in XYZ
Eigen::Matrix<double, 334, 6> get_swing_trajectory(const Eigen::Matrix<double, 3, 1> initial_pos, const Eigen::Matrix<double, 3, 1> middle_pos, const Eigen::Matrix<double, 3, 1> target_pos, 
                                                    const Eigen::Matrix<double, 3, 1> initial_vel, const Eigen::Matrix<double, 3, 1> target_vel, const double duration) {
    Eigen::Matrix<double, 334, 6> trajectory;
    
    for(int i = 0; i < 3; ++i) {
        double a = -2*(duration*initial_vel(i, 0) - duration*target_vel(i, 0) + 4*initial_pos(i, 0) + 4*target_pos(i, 0) - 8*middle_pos(i, 0))/pow(duration,4);
        double b = (5*duration*initial_vel(i, 0) - 3*duration*target_vel(i, 0) + 18*initial_pos(i, 0) + 14*target_pos(i, 0) - 32*middle_pos(i, 0))/pow(duration,3);
        double c = -(4*duration*initial_vel(i, 0) - duration*target_vel(i, 0) + 11*initial_pos(i, 0) + 5*target_pos(i, 0) - 16*middle_pos(i, 0))/pow(duration,2);
        double d = initial_vel(i, 0);
        double e = initial_pos(i, 0);

        int index = 0;
        
        for(double t = 0.0; t < duration; t += duration/334.0) {
            trajectory(index, i+0) = a * pow(t, 4) + b * pow(t, 3) + c * pow(t, 2) + d * t + e;
            trajectory(index, i+3) = 4 * a * pow(t, 3) + 3 * b * pow(t, 2) + 2 * c * t + d;
            ++index;
        }
    }
    
    return trajectory;
}

// Helper function for splitting string by delimiter character
std::vector<std::string> split_string(std::string str, char delimiter) {
    std::vector<std::string> results;

    boost::split(results, str, [&delimiter](char c){return c == delimiter;});

    return results;
}

Eigen::Matrix<double, 5, 1> get_joint_torques(Eigen::Matrix<double, 3, 1> f, double theta1, double theta2, double theta3, double theta4, double theta5, double phi, double theta, double psi, leg_config &config) {
    return (Eigen::Matrix<double, 5, 1>() << f(0, 0)*(config.l2_x*sin(psi + theta1) + config.l2_y*cos(psi + theta1) + config.l3_x*sin(psi + theta1)*cos(theta2) + config.l3_y*cos(psi + theta1) + config.l3_z*sin(theta2)*sin(psi + theta1) + config.l4_x*sin(psi + theta1)*cos(theta2) + config.l4_y*sin(theta2)*sin(theta3)*sin(psi + theta1) + config.l4_y*cos(theta3)*cos(psi + theta1) + config.l4_z*sin(theta2)*sin(psi + theta1)*cos(theta3) - config.l4_z*sin(theta3)*cos(psi + theta1) + config.l5_x*sin(psi + theta1)*cos(theta2) + config.l5_y*sin(theta2)*sin(psi + theta1)*sin(theta3 + theta4) + config.l5_y*cos(psi + theta1)*cos(theta3 + theta4) + config.l5_z*sin(theta2)*sin(psi + theta1)*cos(theta3 + theta4) - config.l5_z*sin(theta3 + theta4)*cos(psi + theta1) + config.l6_x*sin(psi + theta1)*cos(theta2) + config.l6_y*sin(theta2)*sin(psi + theta1)*sin(theta3 + theta4 + theta5) + config.l6_y*cos(psi + theta1)*cos(theta3 + theta4 + theta5) + config.l6_z*sin(theta2)*sin(psi + theta1)*cos(theta3 + theta4 + theta5) - config.l6_z*sin(theta3 + theta4 + theta5)*cos(psi + theta1))*cos(theta) + f(1, 0)*((sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(config.l2_x*cos(theta1) - config.l2_y*sin(theta1) + config.l3_x*cos(theta1)*cos(theta2) - config.l3_y*sin(theta1) + config.l3_z*sin(theta2)*cos(theta1) + config.l4_x*cos(theta1)*cos(theta2) - config.l4_y*(sin(theta1)*cos(theta3) - sin(theta2)*sin(theta3)*cos(theta1)) + config.l4_z*(sin(theta1)*sin(theta3) + sin(theta2)*cos(theta1)*cos(theta3)) + config.l5_x*cos(theta1)*cos(theta2) + config.l5_y*(-sin(theta1)*cos(theta3 + theta4) + sin(theta2)*sin(theta3 + theta4)*cos(theta1)) + config.l5_z*(sin(theta1)*sin(theta3 + theta4) + sin(theta2)*cos(theta1)*cos(theta3 + theta4)) + config.l6_x*cos(theta1)*cos(theta2) + config.l6_y*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)) + config.l6_z*(sin(theta1)*sin(theta3 + theta4 + theta5) + sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5))) + (sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(config.l2_x*sin(theta1) + config.l2_y*cos(theta1) + config.l3_x*sin(theta1)*cos(theta2) + config.l3_y*cos(theta1) + config.l3_z*sin(theta1)*sin(theta2) + config.l4_x*sin(theta1)*cos(theta2) + config.l4_y*(sin(theta1)*sin(theta2)*sin(theta3) + cos(theta1)*cos(theta3)) + config.l4_z*(sin(theta1)*sin(theta2)*cos(theta3) - sin(theta3)*cos(theta1)) + config.l5_x*sin(theta1)*cos(theta2) + config.l5_y*(sin(theta1)*sin(theta2)*sin(theta3 + theta4) + cos(theta1)*cos(theta3 + theta4)) - config.l5_z*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4) + sin(theta3 + theta4)*cos(theta1)) + config.l6_x*sin(theta1)*cos(theta2) + config.l6_y*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5)) - config.l6_z*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) + sin(theta3 + theta4 + theta5)*cos(theta1)))) + f(2, 0)*((sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(config.l2_x*sin(theta1) + config.l2_y*cos(theta1) + config.l3_x*sin(theta1)*cos(theta2) + config.l3_y*cos(theta1) + config.l3_z*sin(theta1)*sin(theta2) + config.l4_x*sin(theta1)*cos(theta2) + config.l4_y*(sin(theta1)*sin(theta2)*sin(theta3) + cos(theta1)*cos(theta3)) + config.l4_z*(sin(theta1)*sin(theta2)*cos(theta3) - sin(theta3)*cos(theta1)) + config.l5_x*sin(theta1)*cos(theta2) + config.l5_y*(sin(theta1)*sin(theta2)*sin(theta3 + theta4) + cos(theta1)*cos(theta3 + theta4)) - config.l5_z*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4) + sin(theta3 + theta4)*cos(theta1)) + config.l6_x*sin(theta1)*cos(theta2) + config.l6_y*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5)) - config.l6_z*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) + sin(theta3 + theta4 + theta5)*cos(theta1))) - (sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(config.l2_x*cos(theta1) - config.l2_y*sin(theta1) + config.l3_x*cos(theta1)*cos(theta2) - config.l3_y*sin(theta1) + config.l3_z*sin(theta2)*cos(theta1) + config.l4_x*cos(theta1)*cos(theta2) - config.l4_y*(sin(theta1)*cos(theta3) - sin(theta2)*sin(theta3)*cos(theta1)) + config.l4_z*(sin(theta1)*sin(theta3) + sin(theta2)*cos(theta1)*cos(theta3)) + config.l5_x*cos(theta1)*cos(theta2) + config.l5_y*(-sin(theta1)*cos(theta3 + theta4) + sin(theta2)*sin(theta3 + theta4)*cos(theta1)) + config.l5_z*(sin(theta1)*sin(theta3 + theta4) + sin(theta2)*cos(theta1)*cos(theta3 + theta4)) + config.l6_x*cos(theta1)*cos(theta2) + config.l6_y*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)) + config.l6_z*(sin(theta1)*sin(theta3 + theta4 + theta5) + sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5)))), f(0, 0)*((-config.l3_x*sin(theta2) + config.l3_z*cos(theta2) - config.l4_x*sin(theta2) + config.l4_y*sin(theta3)*cos(theta2) + config.l4_z*cos(theta2)*cos(theta3) - config.l5_x*sin(theta2) + config.l5_y*sin(theta3 + theta4)*cos(theta2) + config.l5_z*cos(theta2)*cos(theta3 + theta4) - config.l6_x*sin(theta2) + config.l6_y*sin(theta3 + theta4 + theta5)*cos(theta2) + config.l6_z*cos(theta2)*cos(theta3 + theta4 + theta5))*sin(psi)*sin(theta1)*cos(theta) - (-config.l3_x*sin(theta2) + config.l3_z*cos(theta2) - config.l4_x*sin(theta2) + config.l4_y*sin(theta3)*cos(theta2) + config.l4_z*cos(theta2)*cos(theta3) - config.l5_x*sin(theta2) + config.l5_y*sin(theta3 + theta4)*cos(theta2) + config.l5_z*cos(theta2)*cos(theta3 + theta4) - config.l6_x*sin(theta2) + config.l6_y*sin(theta3 + theta4 + theta5)*cos(theta2) + config.l6_z*cos(theta2)*cos(theta3 + theta4 + theta5))*cos(psi)*cos(theta)*cos(theta1) + (config.l3_x*cos(theta2) + config.l3_z*sin(theta2) + config.l4_x*cos(theta2) + config.l4_y*sin(theta2)*sin(theta3) + config.l4_z*sin(theta2)*cos(theta3) + config.l5_x*cos(theta2) + config.l5_y*sin(theta2)*sin(theta3 + theta4) + config.l5_z*sin(theta2)*cos(theta3 + theta4) + config.l6_x*cos(theta2) + config.l6_y*sin(theta2)*sin(theta3 + theta4 + theta5) + config.l6_z*sin(theta2)*cos(theta3 + theta4 + theta5))*sin(theta)) - f(1, 0)*(-(sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(-config.l3_x*sin(theta2) + config.l3_z*cos(theta2) - config.l4_x*sin(theta2) + config.l4_y*sin(theta3)*cos(theta2) + config.l4_z*cos(theta2)*cos(theta3) - config.l5_x*sin(theta2) + config.l5_y*sin(theta3 + theta4)*cos(theta2) + config.l5_z*cos(theta2)*cos(theta3 + theta4) - config.l6_x*sin(theta2) + config.l6_y*sin(theta3 + theta4 + theta5)*cos(theta2) + config.l6_z*cos(theta2)*cos(theta3 + theta4 + theta5))*sin(theta1) + (sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(-config.l3_x*sin(theta2) + config.l3_z*cos(theta2) - config.l4_x*sin(theta2) + config.l4_y*sin(theta3)*cos(theta2) + config.l4_z*cos(theta2)*cos(theta3) - config.l5_x*sin(theta2) + config.l5_y*sin(theta3 + theta4)*cos(theta2) + config.l5_z*cos(theta2)*cos(theta3 + theta4) - config.l6_x*sin(theta2) + config.l6_y*sin(theta3 + theta4 + theta5)*cos(theta2) + config.l6_z*cos(theta2)*cos(theta3 + theta4 + theta5))*cos(theta1) + (config.l3_x*cos(theta2) + config.l3_z*sin(theta2) + config.l4_x*cos(theta2) + config.l4_y*sin(theta2)*sin(theta3) + config.l4_z*sin(theta2)*cos(theta3) + config.l5_x*cos(theta2) + config.l5_y*sin(theta2)*sin(theta3 + theta4) + config.l5_z*sin(theta2)*cos(theta3 + theta4) + config.l6_x*cos(theta2) + config.l6_y*sin(theta2)*sin(theta3 + theta4 + theta5) + config.l6_z*sin(theta2)*cos(theta3 + theta4 + theta5))*sin(phi)*cos(theta)) - f(2, 0)*((sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(-config.l3_x*sin(theta2) + config.l3_z*cos(theta2) - config.l4_x*sin(theta2) + config.l4_y*sin(theta3)*cos(theta2) + config.l4_z*cos(theta2)*cos(theta3) - config.l5_x*sin(theta2) + config.l5_y*sin(theta3 + theta4)*cos(theta2) + config.l5_z*cos(theta2)*cos(theta3 + theta4) - config.l6_x*sin(theta2) + config.l6_y*sin(theta3 + theta4 + theta5)*cos(theta2) + config.l6_z*cos(theta2)*cos(theta3 + theta4 + theta5))*cos(theta1) + (sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(-config.l3_x*sin(theta2) + config.l3_z*cos(theta2) - config.l4_x*sin(theta2) + config.l4_y*sin(theta3)*cos(theta2) + config.l4_z*cos(theta2)*cos(theta3) - config.l5_x*sin(theta2) + config.l5_y*sin(theta3 + theta4)*cos(theta2) + config.l5_z*cos(theta2)*cos(theta3 + theta4) - config.l6_x*sin(theta2) + config.l6_y*sin(theta3 + theta4 + theta5)*cos(theta2) + config.l6_z*cos(theta2)*cos(theta3 + theta4 + theta5))*sin(theta1) - (config.l3_x*cos(theta2) + config.l3_z*sin(theta2) + config.l4_x*cos(theta2) + config.l4_y*sin(theta2)*sin(theta3) + config.l4_z*sin(theta2)*cos(theta3) + config.l5_x*cos(theta2) + config.l5_y*sin(theta2)*sin(theta3 + theta4) + config.l5_z*sin(theta2)*cos(theta3 + theta4) + config.l6_x*cos(theta2) + config.l6_y*sin(theta2)*sin(theta3 + theta4 + theta5) + config.l6_z*sin(theta2)*cos(theta3 + theta4 + theta5))*cos(phi)*cos(theta)), -f(0, 0)*(config.l4_y*sin(theta)*cos(theta2)*cos(theta3) + config.l4_y*sin(theta2)*cos(theta)*cos(theta3)*cos(psi + theta1) + config.l4_y*sin(theta3)*sin(psi + theta1)*cos(theta) - config.l4_z*sin(theta)*sin(theta3)*cos(theta2) - config.l4_z*sin(theta2)*sin(theta3)*cos(theta)*cos(psi + theta1) + config.l4_z*sin(psi + theta1)*cos(theta)*cos(theta3) + config.l5_y*sin(theta)*cos(theta2)*cos(theta3 + theta4) + config.l5_y*sin(theta2)*cos(theta)*cos(psi + theta1)*cos(theta3 + theta4) + config.l5_y*sin(psi + theta1)*sin(theta3 + theta4)*cos(theta) - config.l5_z*sin(theta)*sin(theta3 + theta4)*cos(theta2) - config.l5_z*sin(theta2)*sin(theta3 + theta4)*cos(theta)*cos(psi + theta1) + config.l5_z*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4) + config.l6_y*sin(theta)*cos(theta2)*cos(theta3 + theta4 + theta5) + config.l6_y*sin(theta2)*cos(theta)*cos(psi + theta1)*cos(theta3 + theta4 + theta5) + config.l6_y*sin(psi + theta1)*sin(theta3 + theta4 + theta5)*cos(theta) - config.l6_z*sin(theta)*sin(theta3 + theta4 + theta5)*cos(theta2) - config.l6_z*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta)*cos(psi + theta1) + config.l6_z*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4 + theta5)) - f(1, 0)*((sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(-config.l4_y*(sin(theta1)*sin(theta2)*cos(theta3) - sin(theta3)*cos(theta1)) + config.l4_z*(sin(theta1)*sin(theta2)*sin(theta3) + cos(theta1)*cos(theta3)) + config.l5_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4) + sin(theta3 + theta4)*cos(theta1)) + config.l5_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4) + cos(theta1)*cos(theta3 + theta4)) + config.l6_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) + sin(theta3 + theta4 + theta5)*cos(theta1)) + config.l6_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5))) + (sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(config.l4_y*(sin(theta1)*sin(theta3) + sin(theta2)*cos(theta1)*cos(theta3)) + config.l4_z*(sin(theta1)*cos(theta3) - sin(theta2)*sin(theta3)*cos(theta1)) + config.l5_y*(sin(theta1)*sin(theta3 + theta4) + sin(theta2)*cos(theta1)*cos(theta3 + theta4)) - config.l5_z*(-sin(theta1)*cos(theta3 + theta4) + sin(theta2)*sin(theta3 + theta4)*cos(theta1)) + config.l6_y*(sin(theta1)*sin(theta3 + theta4 + theta5) + sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5)) - config.l6_z*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1))) - (config.l4_y*cos(theta3) - config.l4_z*sin(theta3) + config.l5_y*cos(theta3 + theta4) - config.l5_z*sin(theta3 + theta4) + config.l6_y*cos(theta3 + theta4 + theta5) - config.l6_z*sin(theta3 + theta4 + theta5))*sin(phi)*cos(theta)*cos(theta2)) - f(2, 0)*((sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(config.l4_y*(sin(theta1)*sin(theta3) + sin(theta2)*cos(theta1)*cos(theta3)) + config.l4_z*(sin(theta1)*cos(theta3) - sin(theta2)*sin(theta3)*cos(theta1)) + config.l5_y*(sin(theta1)*sin(theta3 + theta4) + sin(theta2)*cos(theta1)*cos(theta3 + theta4)) - config.l5_z*(-sin(theta1)*cos(theta3 + theta4) + sin(theta2)*sin(theta3 + theta4)*cos(theta1)) + config.l6_y*(sin(theta1)*sin(theta3 + theta4 + theta5) + sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5)) - config.l6_z*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1))) - (sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(-config.l4_y*(sin(theta1)*sin(theta2)*cos(theta3) - sin(theta3)*cos(theta1)) + config.l4_z*(sin(theta1)*sin(theta2)*sin(theta3) + cos(theta1)*cos(theta3)) + config.l5_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4) + sin(theta3 + theta4)*cos(theta1)) + config.l5_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4) + cos(theta1)*cos(theta3 + theta4)) + config.l6_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) + sin(theta3 + theta4 + theta5)*cos(theta1)) + config.l6_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5))) + (config.l4_y*cos(theta3) - config.l4_z*sin(theta3) + config.l5_y*cos(theta3 + theta4) - config.l5_z*sin(theta3 + theta4) + config.l6_y*cos(theta3 + theta4 + theta5) - config.l6_z*sin(theta3 + theta4 + theta5))*cos(phi)*cos(theta)*cos(theta2)), -f(0, 0)*(config.l5_y*sin(theta)*cos(theta2)*cos(theta3 + theta4) + config.l5_y*sin(theta2)*cos(theta)*cos(psi + theta1)*cos(theta3 + theta4) + config.l5_y*sin(psi + theta1)*sin(theta3 + theta4)*cos(theta) - config.l5_z*sin(theta)*sin(theta3 + theta4)*cos(theta2) - config.l5_z*sin(theta2)*sin(theta3 + theta4)*cos(theta)*cos(psi + theta1) + config.l5_z*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4) + config.l6_y*sin(theta)*cos(theta2)*cos(theta3 + theta4 + theta5) + config.l6_y*sin(theta2)*cos(theta)*cos(psi + theta1)*cos(theta3 + theta4 + theta5) + config.l6_y*sin(psi + theta1)*sin(theta3 + theta4 + theta5)*cos(theta) - config.l6_z*sin(theta)*sin(theta3 + theta4 + theta5)*cos(theta2) - config.l6_z*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta)*cos(psi + theta1) + config.l6_z*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4 + theta5)) - f(1, 0)*((sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(config.l5_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4) + sin(theta3 + theta4)*cos(theta1)) + config.l5_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4) + cos(theta1)*cos(theta3 + theta4)) + config.l6_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) + sin(theta3 + theta4 + theta5)*cos(theta1)) + config.l6_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5))) + (sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(config.l5_y*(sin(theta1)*sin(theta3 + theta4) + sin(theta2)*cos(theta1)*cos(theta3 + theta4)) - config.l5_z*(-sin(theta1)*cos(theta3 + theta4) + sin(theta2)*sin(theta3 + theta4)*cos(theta1)) + config.l6_y*(sin(theta1)*sin(theta3 + theta4 + theta5) + sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5)) - config.l6_z*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1))) - (config.l5_y*cos(theta3 + theta4) - config.l5_z*sin(theta3 + theta4) + config.l6_y*cos(theta3 + theta4 + theta5) - config.l6_z*sin(theta3 + theta4 + theta5))*sin(phi)*cos(theta)*cos(theta2)) - f(2, 0)*((sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(config.l5_y*(sin(theta1)*sin(theta3 + theta4) + sin(theta2)*cos(theta1)*cos(theta3 + theta4)) - config.l5_z*(-sin(theta1)*cos(theta3 + theta4) + sin(theta2)*sin(theta3 + theta4)*cos(theta1)) + config.l6_y*(sin(theta1)*sin(theta3 + theta4 + theta5) + sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5)) - config.l6_z*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1))) - (sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(config.l5_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4) + sin(theta3 + theta4)*cos(theta1)) + config.l5_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4) + cos(theta1)*cos(theta3 + theta4)) + config.l6_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) + sin(theta3 + theta4 + theta5)*cos(theta1)) + config.l6_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5))) + (config.l5_y*cos(theta3 + theta4) - config.l5_z*sin(theta3 + theta4) + config.l6_y*cos(theta3 + theta4 + theta5) - config.l6_z*sin(theta3 + theta4 + theta5))*cos(phi)*cos(theta)*cos(theta2)), -f(0, 0)*(config.l6_y*sin(theta)*cos(theta2)*cos(theta3 + theta4 + theta5) + config.l6_y*sin(theta2)*cos(theta)*cos(psi + theta1)*cos(theta3 + theta4 + theta5) + config.l6_y*sin(psi + theta1)*sin(theta3 + theta4 + theta5)*cos(theta) - config.l6_z*sin(theta)*sin(theta3 + theta4 + theta5)*cos(theta2) - config.l6_z*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta)*cos(psi + theta1) + config.l6_z*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4 + theta5)) - f(1, 0)*((config.l6_y*(sin(theta1)*sin(theta3 + theta4 + theta5) + sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5)) - config.l6_z*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)))*(sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi)) + (config.l6_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) + sin(theta3 + theta4 + theta5)*cos(theta1)) + config.l6_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5)))*(sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi)) - (config.l6_y*cos(theta3 + theta4 + theta5) - config.l6_z*sin(theta3 + theta4 + theta5))*sin(phi)*cos(theta)*cos(theta2)) - f(2, 0)*((config.l6_y*(sin(theta1)*sin(theta3 + theta4 + theta5) + sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5)) - config.l6_z*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)))*(sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi)) - (config.l6_y*(-sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) + sin(theta3 + theta4 + theta5)*cos(theta1)) + config.l6_z*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5)))*(sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi)) + (config.l6_y*cos(theta3 + theta4 + theta5) - config.l6_z*sin(theta3 + theta4 + theta5))*cos(phi)*cos(theta)*cos(theta2))).finished();
}

std::string getLogTypeString(LogType type) {
    static const std::string LogTypeNames[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    return LogTypeNames[type];
}

void log(std::string message, LogType type) {
    ofstream log_file;
    char* pPath;
    pPath = getenv ("IS_DOCKER");
    if (pPath == "Y" || pPath == "YES") {
        log_file.open("/plot_data/controller_log.txt", ios::app);
    }
    else {
        log_file.open("../.././plot_data/controller_log.txt", ios::app);
    }

    log_file << "[" << boost::posix_time::second_clock::local_time().time_of_day() << "] [" << getLogTypeString(type) << "]" << ": " << message << std::endl;
    
    log_file.close();
}

void print_threadsafe(std::string str, std::string sender, LogType type, bool log_to_file) {
    std::string prepared_string = "\033[1;36m[From '" + sender + "']:\033[0m \033[1;33m'" + str + "'\033[0m\n";
    std::cout << prepared_string;

    if(log_to_file) {
        std::string log_string = "[From '" + sender + "']: '" + str + "'";
        std::replace(log_string.begin(), log_string.end(), '\n', '\t');

        log(log_string, type);
    }
}