#ifndef LEG_H
#define LEG_H

#include <iostream>
#include <dirent.h>
#include <typeinfo>

#include <chrono>
#include <thread>
#include <functional>

#include <fstream>

#include "nameof.h"

#include <string>
#include <random>
#include <ctime>
#include <cmath>

#include <stdlib.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <unistd.h>

#include <errno.h> //It defines macros for reporting and retrieving error conditions through error codes
#include <time.h> //contains various functions for manipulating date and time
#include <unistd.h> //contains various constants
#include <sys/types.h> //contains a number of basic derived types that should be used whenever appropriate
#include <arpa/inet.h> // defines in_addr structure
#include <sys/socket.h> // for socket creation
#include <netinet/in.h> //contains constants and structures needed for internet domain addresses

#include <iomanip>

#include "leg_config.hpp"
#include "model_functions.hpp"
#include "Helpers.hpp"
#include "log_type.hpp"

using Eigen::MatrixXd;
using namespace std;

class Leg {
    public: Leg(double hip_offset_x, double hip_offset_y, double hip_offset_z);
    
    public: long long iteration_counter;

    public: double hip_offset_x;
    public: double hip_offset_y;
    public: double hip_offset_z;

    public: double theta1;
    public: double theta2;
    public: double theta3;
    public: double theta4;
    public: double theta5;

    public: double theta1dot;
    public: double theta2dot;
    public: double theta3dot;
    public: double theta4dot;
    public: double theta5dot;

    public: double trajectory_start_time;
    public: double t_stance_remainder;

    // Boolean representing foot state. True means foot is in the air, i.e. no contact, false means foot is in stance phase. i.e. contact
    public: bool swing_phase;
    
    public: std::mutex q_mutex, q_dot_mutex, foot_pos_world_mutex, foot_pos_world_desired_mutex, lift_off_pos_mutex, lift_off_vel_mutex, t_stance_remainder_mutex, foot_pos_body_frame_mutex,
                        trajectory_start_time_mutex, foot_trajectory_mutex, foot_pos_desired_world_mutex;

    public: Eigen::Matrix<double, 5, 1> q; // Leg angle vector / Model state
    public: Eigen::Matrix<double, 5, 1> q_dot; // Leg angular velocity vector / Differentiated model state

    public: Eigen::Matrix<double, 5, 1> C; // matrix containing the result of C * q_dot and the other terms based on the jupyter notebook
    public: Eigen::Matrix<double, 5, 5> B; // mass and inertia matrix of the leg model
    public: Eigen::Matrix<double, 5, 1> G; // gravity vector of the leg model. If directly applied as torques to each joint, it should compensate for gravity.

    public: Eigen::Matrix<double, 3, 5> J_foot; // Jacobian of the foot / end effector, also called the contact Jacobian.
    public: Eigen::Matrix<double, 5, 5> J_foot_combined; // Combined Jacobian with geometric positional part and analytical orientation part.
    public: Eigen::Matrix<double, 3, 5> J_foot_dot; // Time derivative of the contact / end effector Jacobian,

    public: Eigen::Matrix<double, 3, 3> Lambda; // "Desired Inertia matrix" of the leg, based on Jacobian and inertia matrix B / M

    public: Eigen::Matrix<double, 5, 5> Kp; // Cartesian Position gain matrix for calculation of torque setpoint
    public: Eigen::Matrix<double, 5, 5> Kd; // Derivative / Cartesian Velocity for calculation of torque setpoint

    public: Eigen::Matrix<double, 5, 1> tau_ff; // Vector containing feedforward torque based on Coriolis, Centrifugal, gravity and feed-forward acceleration terms.
    public: Eigen::Matrix<double, 5, 1> tau_setpoint; // Final torque setpoint calculated from above matrices and feedforward torque added.

    public: Eigen::Matrix<double, 5, 1> foot_pos; // Cartesian foot / end-effector position, hip frame
    public: Eigen::Matrix<double, 3, 1> foot_pos_body_frame; // Body frame
    public: Eigen::Matrix<double, 3, 1> foot_pos_desired_body_frame; // Body frame
    public: Eigen::Matrix<double, 3, 1> foot_pos_desired_world; // World frame
    public: Eigen::Matrix<double, 3, 1> foot_pos_world; // foot position in world frame
    public: Eigen::Matrix<double, 3, 1> lift_off_pos; // Body frame
    public: Eigen::Matrix<double, 3, 1> foot_pos_world_discretization; // World frame

    public: Eigen::Matrix<double, 5, 1> foot_vel; // Cartesian foot/ end-effector velocity
    public: Eigen::Matrix<double, 3, 1> lift_off_vel; // Body frame

    public: Eigen::Matrix<double, 5, 1> pos_desired; // Desired cartesian foot / end-effector position + orientation (roll and yaw)
    public: Eigen::Matrix<double, 5, 1> vel_desired; // Desired cartesian foot / end-effector velocity + angular velocity
    public: Eigen::Matrix<double, 3, 1> accel_desired; // Desired cartesian foot / end-effector acceleration

    public: Eigen::Matrix<double, 3, 3> h; // Damping ratio matrix
    public: Eigen::Matrix<double, 3, 1> omega_desired; // Desired natural frequency of the leg

    public: Eigen::Matrix<double, 334, 6> foot_trajectory;


    public: double Kp_orientation;
    public: double Kd_orientation;

    // Euler Angle definitions:
    // roll - around x - alpha - phi
    // pitch - around y - beta - theta
    // yaw - around z - gamma - psi

    public: double phi = 0; // Roll, rotation around X, alpha
    public: double theta = 0; // Pitch, rotation around Y, beta
    public: double psi = 0; // Yaw, rotation arond Z, gamma

    public: leg_config config;

    public: void update_torque_setpoint();

    public: void update_foot_pos_body_frame(Eigen::Matrix<double, 13, 1> &com_state);

    public: void update_foot_trajectory(Eigen::Matrix<double, 13, 1> &com_state, Eigen::Matrix<double, 3, 1> next_body_vel, double t_stance, double time);

    public: void update();
};

#endif