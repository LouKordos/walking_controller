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
#include "CartesianTrajectory.hpp"
#include "Helpers.hpp"
#include "log_type.hpp"
#include "ContactState.hpp"

using Eigen::MatrixXd;
using namespace std;

class Leg {
    public: Leg(double hip_offset_x, double hip_offset_y, double hip_offset_z, int contact_state_port);
    
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

    private: double trajectory_start_time;

    // Boolean representing foot state. True means foot is in the air, i.e. no contact, false means foot is in stance phase. i.e. contact
    private: bool swing_phase;
    
    private: std::mutex q_mutex, q_dot_mutex, foot_pos_world_desired_mutex, lift_off_pos_mutex, lift_off_vel_mutex, foot_pos_body_frame_mutex,
                        trajectory_start_time_mutex, foot_trajectory_mutex, next_foot_pos_world_desired_mutex, foot_pos_desired_body_frame_mutex, swing_phase_mutex;
    
    private: Eigen::Matrix<double, 5, 1> q; // Leg angle vector / Model state
    private: Eigen::Matrix<double, 5, 1> q_dot; // Leg angular velocity vector / Differentiated model state

    private: Eigen::Matrix<double, 5, 1> C; // matrix containing the result of C * q_dot and the other terms based on the jupyter notebook
    private: Eigen::Matrix<double, 5, 5> B; // mass and inertia matrix of the leg model
    private: Eigen::Matrix<double, 5, 1> G; // gravity vector of the leg model. If directly applied as torques to each joint, it should compensate for gravity.

    private: Eigen::Matrix<double, 3, 5> J_foot; // Jacobian of the foot / end effector, also called the contact Jacobian.
    private: Eigen::Matrix<double, 5, 5> J_foot_combined; // Combined Jacobian with geometric positional part and analytical orientation part.
    private: Eigen::Matrix<double, 3, 5> J_foot_dot; // Time derivative of the contact / end effector Jacobian,

    private: Eigen::Matrix<double, 3, 3> Lambda; // "Desired Inertia matrix" of the leg, based on Jacobian and inertia matrix B / M

    private: Eigen::Matrix<double, 5, 5> Kp; // Cartesian Position gain matrix for calculation of torque setpoint
    private: Eigen::Matrix<double, 5, 5> Kd; // Derivative / Cartesian Velocity for calculation of torque setpoint

    private: Eigen::Matrix<double, 5, 1> tau_ff; // Vector containing feedforward torque based on Coriolis, Centrifugal, gravity and feed-forward acceleration terms.
    public: Eigen::Matrix<double, 5, 1> tau_setpoint; // Final torque setpoint calculated from above matrices and feedforward torque added.

    public: Eigen::Matrix<double, 5, 1> foot_pos; // Cartesian foot / end-effector position, hip frame
    private: Eigen::Matrix<double, 3, 1> foot_pos_body_frame; // Foot / end-effector position in body frame
    private: Eigen::Matrix<double, 3, 1> foot_pos_desired_body_frame; // Body frame
    private: Eigen::Matrix<double, 3, 1> next_foot_pos_world_desired; // Next desired foot position in world frame, obtained from the MPC prediction horizon at the next contact swap. It is also the target position for the trajectory planner.
    private: Eigen::Matrix<double, 3, 1> foot_pos_world_desired; // Most recent desired foot position in world frame, calculated by the MPC formula using Raibert heuristic etc.
    private: Eigen::Matrix<double, 3, 1> lift_off_pos; // Body frame
    public: Eigen::Matrix<double, 3, 1> foot_pos_world_discretization; // World frame

    public: Eigen::Matrix<double, 5, 1> foot_vel; // Cartesian foot/ end-effector velocity
    private: Eigen::Matrix<double, 3, 1> lift_off_vel; // Body frame

    public: Eigen::Matrix<double, 5, 1> pos_desired; // Desired cartesian foot / end-effector position + orientation (roll and yaw)
    public: Eigen::Matrix<double, 5, 1> vel_desired; // Desired cartesian foot / end-effector velocity + angular velocity
    public: Eigen::Matrix<double, 3, 1> accel_desired; // Desired cartesian foot / end-effector acceleration

    public: Eigen::Matrix<double, 3, 3> h; // Damping ratio matrix
    public: Eigen::Matrix<double, 3, 1> omega_desired; // Desired natural frequency of the leg

    public: CartesianTrajectory foot_trajectory;
    public: ContactState contactState;
    
    public: Eigen::Matrix<double, 4, 4> H_hip_body; // Matrix for transforming a point from hip frame to body frame. Dependent on hip_offset values passed as arguments to Leg constructor

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

    public: void set_q(const Eigen::Matrix<double, 5, 1> q);
    
    public: void set_q(double theta1, double theta2, double theta3, double theta4, double theta5);

    public: Eigen::Matrix<double, 5, 1> get_q();

    public: void set_q_dot(const Eigen::Matrix<double, 5, 1> q_dot);
    
    public: void set_q_dot(double theta1_dot, double theta2_dot, double theta3_dot, double theta4_dot, double theta5_dot);

    public: Eigen::Matrix<double, 5, 1> get_q_dot();

    public: double get_trajectory_start_time();

    public: void set_trajectory_start_time(const double t);

    public: void set_swing_phase(const bool swing);
    
    public: bool get_swing_phase();

    public: void update_torque_setpoint();

    public: void update_foot_pos_body_frame(const Eigen::Matrix<double, 13, 1> &com_state);

    public: Eigen::Matrix<double, 3, 1> get_foot_pos_body_frame();

    public: Eigen::Matrix<double, 3, 1> get_next_foot_pos_world_desired();

    public: void set_next_foot_pos_world_desired(const Eigen::Matrix<double, 3, 1> next_foot_pos_world_desired);

    public: Eigen::Matrix<double, 3, 1> get_foot_pos_world_desired();

    public: void set_foot_pos_world_desired(const Eigen::Matrix<double, 3, 1> foot_pos_world_desired);

    public: Eigen::Matrix<double, 3, 1> get_lift_off_pos();

    public: void set_lift_off_pos(const Eigen::Matrix<double, 3, 1> lift_off_pos);

    public: Eigen::Matrix<double, 3, 1> get_lift_off_vel();

    public: void set_lift_off_vel(const Eigen::Matrix<double, 3, 1> lift_off_vel);

    public: void update_foot_trajectory(Eigen::Matrix<double, 13, 1> &com_state, Eigen::Matrix<double, 3, 1> next_body_vel, double t_stance, double time);

    public: void update();

    public: Eigen::Matrix<double, 3, 1> get_foot_pos_world(Eigen::Matrix<double, 13, 1> &com_state);
};

#endif