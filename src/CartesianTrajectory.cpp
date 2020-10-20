#include "include/CartesianTrajectory.hpp"

CartesianTrajectory::CartesianTrajectory() {
    
}

Eigen::Matrix<double, 3, 1> CartesianTrajectory::get_trajectory_pos(double t) {
    Eigen::Matrix<double, 3, 1> temp_pos;

    for(int i = 0; i < 3; ++i) {
        double a = coefficients(i).a;
        double b = coefficients(i).b;
        double c = coefficients(i).c;
        double d = coefficients(i).d;
        double e = coefficients(i).e;

        temp_pos(i, 0) = a * pow(t, 4) + b * pow(t, 3) + c * pow(t, 2) + d * t + e;
    }

    return temp_pos;
}

Eigen::Matrix<double, 3, 1> CartesianTrajectory::get_trajectory_vel(double t) {
    Eigen::Matrix<double, 3, 1> temp_vel;

    for(int i = 0; i < 3; ++i) {
        double a = coefficients(i).a;
        double b = coefficients(i).b;
        double c = coefficients(i).c;
        double d = coefficients(i).d;
        
        temp_vel(i, 0) = 4 * a * pow(t, 3) + 3 * b * pow(t, 2) + 2 * c * t + d;
    }

    return temp_vel;    
}

Eigen::Matrix<double, 3, 1> CartesianTrajectory::get_trajectory_accel(double t) {
    Eigen::Matrix<double, 3, 1> temp_accel;

    for(int i = 0; i < 3; ++i) {
        double a = coefficients(i).a;
        double b = coefficients(i).b;
        double c = coefficients(i).c;

        temp_accel(i, 0) = 12 * a * pow(t, 2) + 6 * b * t + 2 * c;
    }

    return temp_accel;
}