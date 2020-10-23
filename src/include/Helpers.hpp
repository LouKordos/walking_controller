#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <mutex>
#include <vector>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "log_type.hpp"
#include "leg_config.hpp"

using Eigen::MatrixXd;
using namespace std;

void constrain(double &value, double lower_limit, double upper_limit);

void constrain_int(int &value, int lower_limit, int upper_limit);

unsigned long long factorial(long n);

Eigen::Matrix<double, 334, 6> get_swing_trajectory(const Eigen::Matrix<double, 3, 1> initial_pos, const Eigen::Matrix<double, 3, 1> middle_pos, const Eigen::Matrix<double, 3, 1> target_pos, 
                                                    const Eigen::Matrix<double, 3, 1> initial_vel, const Eigen::Matrix<double, 3, 1> target_vel, const double duration);

std::vector<std::string> split_string(std::string str, char delimiter);

Eigen::Matrix<double, 5, 1> get_joint_torques(Eigen::Matrix<double, 3, 1> f, double theta1, double theta2, double theta3, double theta4, double theta5, double phi, double theta, double psi, leg_config &config);

std::string getLogTypeString(LogType type);

void log(std::string message, LogType type);

void print_threadsafe(std::string str, std::string sender, LogType type, bool log_to_file = true);

#endif