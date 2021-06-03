#include "include/CartesianTrajectory.hpp"

CartesianTrajectory::CartesianTrajectory() {
    
}

Eigen::Matrix<double, 3, 1> CartesianTrajectory::get_position(double t) {
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

Eigen::Matrix<double, 3, 1> CartesianTrajectory::get_velocity(double t) {
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

Eigen::Matrix<double, 3, 1> CartesianTrajectory::get_acceleration(double t) {
    Eigen::Matrix<double, 3, 1> temp_accel;

    for(int i = 0; i < 3; ++i) {
        double a = coefficients(i).a;
        double b = coefficients(i).b;
        double c = coefficients(i).c;

        temp_accel(i, 0) = 12 * a * pow(t, 2) + 6 * b * t + 2 * c;
    }

    return temp_accel;
}

void CartesianTrajectory::update(const Eigen::Matrix<double, 3, 1> initial_pos, const Eigen::Matrix<double, 3, 1> middle_pos, const Eigen::Matrix<double, 3, 1> target_pos, const Eigen::Matrix<double, 3, 1> initial_vel, const Eigen::Matrix<double, 3, 1> target_vel, const double duration) {
    for(int i = 0; i < 3; ++i) {
        double a = -2*(duration*initial_vel(i, 0) - duration*target_vel(i, 0) + 4*initial_pos(i, 0) + 4*target_pos(i, 0) - 8*middle_pos(i, 0))/pow(duration,4);
        double b = (5*duration*initial_vel(i, 0) - 3*duration*target_vel(i, 0) + 18*initial_pos(i, 0) + 14*target_pos(i, 0) - 32*middle_pos(i, 0))/pow(duration,3);
        double c = -(4*duration*initial_vel(i, 0) - duration*target_vel(i, 0) + 11*initial_pos(i, 0) + 5*target_pos(i, 0) - 16*middle_pos(i, 0))/pow(duration,2);
        double d = initial_vel(i, 0);
        double e = initial_pos(i, 0);
        coefficients(i) = PolyCoefficients4D(a, b, c, d, e);
    }
}
