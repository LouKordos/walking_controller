
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
    
    public: void update(const Eigen::Matrix<double, 3, 1> initial_pos, const Eigen::Matrix<double, 3, 1> middle_pos, const Eigen::Matrix<double, 3, 1> target_pos, const Eigen::Matrix<double, 3, 1> initial_vel, const Eigen::Matrix<double, 3, 1> target_vel, const double duration);

    public: Eigen::Matrix<double, 3, 1> get_trajectory_pos(double t);

    public: Eigen::Matrix<double, 3, 1> get_trajectory_vel(double t);

    public: Eigen::Matrix<double, 3, 1> get_trajectory_accel(double t);
};