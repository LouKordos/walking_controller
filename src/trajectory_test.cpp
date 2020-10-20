#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>
#include <vector>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>

using namespace std;
using namespace std::chrono;
using Eigen::MatrixXd;

#include "include/CartesianTrajectory.hpp"

int main() {
    high_resolution_clock::time_point start = high_resolution_clock::now();
    auto traj = new CartesianTrajectory();
    for(int i = 0; i < 10000; ++i) {
        traj->update((Eigen::Matrix<double,3,1>() << 0.1, 0.1, -1).finished(), (Eigen::Matrix<double,3,1>() << 0.05, 0.05, -0.9).finished(), 
                                        (Eigen::Matrix<double,3,1>() << 0, 0, -1).finished(), 
                                        (Eigen::Matrix<double,3,1>() << 0.0, 0.0, 0).finished(), (Eigen::Matrix<double,3,1>() << 0.0, 0.0, -0).finished(), 10);

        std::cout << traj->get_trajectory_pos(i * 0.0001) << std::endl;
        std::cout << traj->get_trajectory_vel(i * 0.0001) << std::endl;
        std::cout << traj->get_trajectory_accel(i * 0.0001) << std::endl;

        ofstream traj_log_file;
        traj_log_file.open("./plot_data.csv", ios::app);
        traj_log_file << traj->get_trajectory_pos(i * 0.0001)(0, 0) << "," << traj->get_trajectory_pos(i * 0.0001)(1, 0) << "," << traj->get_trajectory_pos(i * 0.0001)(2, 0)
        << "," << traj->get_trajectory_vel(i * 0.0001)(0, 0) << "," << traj->get_trajectory_vel(i * 0.0001)(1, 0) << "," << traj->get_trajectory_vel(i * 0.0001)(2, 0) 
        << "," << traj->get_trajectory_accel(i * 0.0001)(0, 0) << "," << traj->get_trajectory_accel(i * 0.0001)(1, 0) << "," << traj->get_trajectory_accel(i * 0.0001)(2, 0)
        << std::endl;
        traj_log_file.close();
    }

    high_resolution_clock::time_point end = high_resolution_clock::now();

    double duration =  duration_cast<microseconds>(end - start).count();
    std::cout << duration / 10000 << std::endl;

    return 0;
}