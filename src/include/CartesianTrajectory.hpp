
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>
#include <iostream>
#include <cmath>

#include "PolyCoefficients4D.hpp"

using Eigen::MatrixXd;
using namespace std;

class CartesianTrajectory {
    public: CartesianTrajectory();

    private: Eigen::Matrix<PolyCoefficients4D, 3, 1> coefficients;
    
    public: void update(const Eigen::Matrix<double, 3, 1> initial_pos, const Eigen::Matrix<double, 3, 1> middle_pos, const Eigen::Matrix<double, 3, 1> target_pos, const Eigen::Matrix<double, 3, 1> initial_vel, const Eigen::Matrix<double, 3, 1> target_vel, const double duration) {
        for(int i = 0; i < 3; ++i) {
            double a = -2*(duration*initial_vel(i, 0) - duration*target_vel(i, 0) + 4*initial_pos(i, 0) + 4*target_pos(i, 0) - 8*middle_pos(i, 0))/pow(duration,4);
            double b = (5*duration*initial_vel(i, 0) - 3*duration*target_vel(i, 0) + 18*initial_pos(i, 0) + 14*target_pos(i, 0) - 32*middle_pos(i, 0))/pow(duration,3);
            double c = -(4*duration*initial_vel(i, 0) - duration*target_vel(i, 0) + 11*initial_pos(i, 0) + 5*target_pos(i, 0) - 16*middle_pos(i, 0))/pow(duration,2);
            double d = initial_vel(i, 0);
            double e = initial_pos(i, 0);
            coefficients(i) = PolyCoefficients4D(a, b, c, d, e);
        }
    }

    public: Eigen::Matrix<double, 3, 1> get_trajectory_pos(double t);

    public: Eigen::Matrix<double, 3, 1> get_trajectory_vel(double t);

    public: Eigen::Matrix<double, 3, 1> get_trajectory_accel(double t);
};