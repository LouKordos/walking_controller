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

#include <unistd.h>
// #include <zcm/zcm-cpp.hpp>
//#include <sys/types .h>

//#include "leg_state.hpp"
//#include "torque_setpoint.hpp"

#include <errno.h> //It defines macros for reporting and retrieving error conditions through error codes
#include <time.h> //contains various functions for manipulating date and time
#include <unistd.h> //contains various constants
#include <sys/types.h> //contains a number of basic derived types that should be used whenever appropriate
#include <arpa/inet.h> // defines in_addr structure
#include <sys/socket.h> // for socket creation
#include <netinet/in.h> //contains constants and structures needed for internet domain addresses

#include <iomanip>
#include "casadi/casadi.hpp"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using namespace casadi;

#include "model_functions.cpp"

using namespace std;
using namespace std::chrono;

static const int left_leg_torque_port = 4200;
static const int right_leg_torque_port = 4201;
static const int mpc_port = 4801;

static const int udp_buffer_size = 4096; // Buffer size for receiving leg state from gazebosim

// Booleans representing foot state. True means foot is in the air, i.e. no contact, false means foot is in stance phase. i.e. contact
static bool swing_left = false;
static bool swing_right = true;

static const int n = 13; // Number of states in model
static const int m = 6; // Number of control input variables in model

static const double dt = 1/30.0; // Sampling interval, Timestep length in seconds
static const int N = 20; // MPC Prediction Horizon Length in Number of Samples

double f_min_z = 0; // Min contact Force in Z direction for MPC constraint, limits X and Y forces through friction constraint
double f_max_z = 1200; // Max contact Force in Z direction for MPC constraint, limits X and Y forces through friction constraint

static const double m_value = 30.0; // Torso (and eventually leg) mass in kg

static double t_stance_remainder_left, t_stance_remainder_right;

const int contact_swap_interval = 10; // Interval at which the contact swaps from one foot to the other in Samples
double t_stance = contact_swap_interval * dt; // Duration that the foot will be in stance phase

static const double hip_offset_left_leg = -0.15;
static const double hip_offset_right_leg = 0.15;

static Eigen::Matrix<double, n, 1> x_t = (Eigen::Matrix<double, n, 1>() << 0., 0., 0., 0, 0, 0.8, 0, 0, 0, 0, 0, 0, -9.81).finished();
static Eigen::Matrix<double, m, 1> u_t = (Eigen::Matrix<double, m, 1>() << 0, 0, m_value*9.81 / 2, 0, 0, m_value*9.81/2).finished();

static Eigen::Matrix<double, 3, 1> left_foot_pos_world = Eigen::ArrayXd::Zero(3, 1); // Left foot position in world frame
static Eigen::Matrix<double, 3, 1> right_foot_pos_world = Eigen::ArrayXd::Zero(3, 1); // Right foot position in world frame

static Eigen::Matrix<double, 3, 1> left_foot_pos_desired_world = Eigen::ArrayXd::Zero(3, 1);
static Eigen::Matrix<double, 3, 1> right_foot_pos_desired_world = Eigen::ArrayXd::Zero(3, 1);

static Eigen::Matrix<double, 3, 1> lift_off_pos_left = Eigen::ArrayXd::Zero(3, 1);
static Eigen::Matrix<double, 3, 1> lift_off_pos_right = Eigen::ArrayXd::Zero(3, 1);

static Eigen::Matrix<double, 3, 1> lift_off_vel_left = Eigen::ArrayXd::Zero(3, 1);
static Eigen::Matrix<double, 3, 1> lift_off_vel_right = Eigen::ArrayXd::Zero(3, 1);

static Eigen::Matrix<double, 300, 6> foot_trajectory_left = Eigen::ArrayXXd::Zero(300, 6);
static Eigen::Matrix<double, 300, 6> foot_trajectory_right = Eigen::ArrayXXd::Zero(300, 6);

static Eigen::Matrix<double, 3, 1> next_body_vel = Eigen::ArrayXd::Zero(3, 1);

std::mutex x_mutex, u_mutex,
            left_foot_pos_world_mutex, right_foot_pos_world_mutex, 
            left_foot_pos_desired_world_mutex, right_foot_pos_desired_world_mutex,
            lift_off_pos_left_mutex, lift_off_pos_right_mutex,
            lift_off_vel_left_mutex, lift_off_vel_right_mutex,
            t_stance_remainder_left_mutex, t_stance_remainder_right_mutex,
            foot_trajectory_left_mutex, foot_trajectory_right_mutex,
            next_body_vel_mutex;

// Setting up debugging and plotting csv file

int largest_index = 0;
std::string filename;

// Helper function for splitting string by delimiter character
std::vector<std::string> split_string(std::string str, char delimiter) {
    std::vector<std::string> results;

    boost::split(results, str, [&delimiter](char c){return c == delimiter;});

    return results;
}

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

//TODO: Make trajectory matrix length dynamic, it is currently 1 second long, assuming 1ms time steps
Eigen::Matrix<double, 300, 6> get_swing_trajectory(const Eigen::Matrix<double, 3, 1> initial_pos, const Eigen::Matrix<double, 3, 1> middle_pos, const Eigen::Matrix<double, 3, 1> target_pos, 
                                                    const Eigen::Matrix<double, 3, 1> initial_vel, const Eigen::Matrix<double, 3, 1> target_vel, const double duration) {
    Eigen::Matrix<double, 300, 6> trajectory;
    
    for(int i = 0; i < 3; ++i) {
        double a = -2*(duration*initial_vel(i, 0) - duration*target_vel(i, 0) + 4*initial_pos(i, 0) + 4*target_pos(i, 0) - 8*middle_pos(i, 0))/pow(duration,4);
        double b = (5*duration*initial_vel(i, 0) - 3*duration*target_vel(i, 0) + 18*initial_pos(i, 0) + 14*target_pos(i, 0) - 32*middle_pos(i, 0))/pow(duration,3);
        double c = -(4*duration*initial_vel(i, 0) - duration*target_vel(i, 0) + 11*initial_pos(i, 0) + 5*target_pos(i, 0) - 16*middle_pos(i, 0))/pow(duration,2);
        double d = initial_vel(i, 0);
        double e = initial_pos(i, 0);

        int index = 0;

        for(double t = 0.0; t < duration; t += 1/300.0) {
            trajectory(index, i*2+0) = a * pow(t, 4) + b * pow(t, 3) + c * pow(t, 2) + d * t + e;
            trajectory(index, i*2+1) = 4 * a * pow(t, 3) + 3 * b * pow(t, 2) + 2 * c * t + d;
            ++index;
        }
    }

    return trajectory;
}

void update_left_leg_foot_trajectory() {
    x_mutex.lock();
    Eigen::Matrix<double, n, 1> x = x_t;
    x_mutex.unlock();

    double phi_com = x(0, 0);
    double theta_com = x(1, 0);
    double psi_com = x(2, 0);

    double pos_x_com = x(3, 0);
    double pos_y_com = x(4, 0);
    double pos_z_com = x(5, 0);

    double vel_x_com = x(9, 0);
    double vel_y_com = x(10, 0);
    double vel_z_com = x(11, 0);

    Eigen::Matrix<double, 4, 4> H_world_hip = (Eigen::Matrix<double, 4, 4>() << cos(psi_com)*cos(theta_com), sin(psi_com)*cos(theta_com), -sin(theta_com), -pos_y_com*sin(psi_com)*cos(theta_com) + pos_z_com*sin(theta_com) - (hip_offset_left_leg + pos_x_com)*cos(psi_com)*cos(theta_com),
                                                        sin(phi_com)*sin(theta_com)*cos(psi_com) - sin(psi_com)*cos(phi_com), sin(phi_com)*sin(psi_com)*sin(theta_com) + cos(phi_com)*cos(psi_com), sin(phi_com)*cos(theta_com), -pos_y_com*sin(phi_com)*sin(psi_com)*sin(theta_com) - pos_y_com*cos(phi_com)*cos(psi_com) - pos_z_com*sin(phi_com)*cos(theta_com) - (hip_offset_left_leg + pos_x_com)*sin(phi_com)*sin(theta_com)*cos(psi_com) + (hip_offset_left_leg + pos_x_com)*sin(psi_com)*cos(phi_com), 
                                                        sin(phi_com)*sin(psi_com) + sin(theta_com)*cos(phi_com)*cos(psi_com), -sin(phi_com)*cos(psi_com) + sin(psi_com)*sin(theta_com)*cos(phi_com), cos(phi_com)*cos(theta_com), pos_y_com*sin(phi_com)*cos(psi_com) - pos_y_com*sin(psi_com)*sin(theta_com)*cos(phi_com) - pos_z_com*cos(phi_com)*cos(theta_com) - (hip_offset_left_leg + pos_x_com)*sin(phi_com)*sin(psi_com) - (hip_offset_left_leg + pos_x_com)*sin(theta_com)*cos(phi_com)*cos(psi_com),
                                                        0, 0, 0, 1).finished();
        
    //Inverse of H_body_world, see Point Mass Jupyter noteboo for calculation
    Eigen::Matrix<double, 4, 4> H_world_body = (Eigen::Matrix<double, 4, 4>() << cos(psi_com)*cos(theta_com), sin(psi_com)*cos(theta_com), -sin(theta_com), -pos_x_com*cos(psi_com)*cos(theta_com) - pos_y_com*sin(psi_com)*cos(theta_com) + pos_z_com*sin(theta_com), 
                                                sin(phi_com)*sin(theta_com)*cos(psi_com) - sin(psi_com)*cos(phi_com), sin(phi_com)*sin(psi_com)*sin(theta_com) + cos(phi_com)*cos(psi_com), sin(phi_com)*cos(theta_com), -pos_x_com*sin(phi_com)*sin(theta_com)*cos(psi_com) + pos_x_com*sin(psi_com)*cos(phi_com) - pos_y_com*sin(phi_com)*sin(psi_com)*sin(theta_com) - pos_y_com*cos(phi_com)*cos(psi_com) - pos_z_com*sin(phi_com)*cos(theta_com), 
                                                sin(phi_com)*sin(psi_com) + sin(theta_com)*cos(phi_com)*cos(psi_com), -sin(phi_com)*cos(psi_com) + sin(psi_com)*sin(theta_com)*cos(phi_com), cos(phi_com)*cos(theta_com), -pos_x_com*sin(phi_com)*sin(psi_com) - pos_x_com*sin(theta_com)*cos(phi_com)*cos(psi_com) + pos_y_com*sin(phi_com)*cos(psi_com) - pos_y_com*sin(psi_com)*sin(theta_com)*cos(phi_com) - pos_z_com*cos(phi_com)*cos(theta_com), 
                                                0, 0, 0, 1).finished();
    
    left_foot_pos_desired_world_mutex.lock();
    Eigen::Matrix<double, 3, 1> pos_desired_left_leg_body_frame = (H_world_body * (Eigen::Matrix<double, 4, 1>() << left_foot_pos_desired_world, 1).finished()).block<3, 1>(0, 0);
    left_foot_pos_desired_world_mutex.unlock();

    // We need the foot position in the body frame. The below is the dirty equivalent of multiplying with a transformation matrix that has identity for the rotation part and hip_offset_left_leg for x displacement
    Eigen::Matrix<double, 3, 1> foot_pos_left_leg_body_frame = (Eigen::Matrix<double, 3, 1>() << foot_pos_left_leg(0, 0) + hip_offset_left_leg, foot_pos_left_leg(1, 0), foot_pos_left_leg(2, 0)).finished();
    
    double step_height_world = 0.15;

    double step_height_body = (H_world_body * (Eigen::Matrix<double, 4, 1>() << 0, 0, step_height_world, 1).finished())(2, 0);

    next_body_vel_mutex.lock();
    lift_off_pos_left_mutex.lock();
    lift_off_vel_left_mutex.lock();
    foot_trajectory_left_mutex.lock();
    
    foot_trajectory_left = get_swing_trajectory(lift_off_pos_left, 
        (Eigen::Matrix<double, 3, 1>() << (lift_off_pos_left(0, 0) - pos_desired_left_leg_body_frame(0, 0)) / 2, (lift_off_pos_left(1, 0) - pos_desired_left_leg_body_frame(1, 0)) / 2, step_height_body).finished(), pos_desired_left_leg_body_frame, 
        lift_off_vel_left, -next_body_vel, 
        t_stance);
    foot_trajectory_left_mutex.unlock();
    next_body_vel_mutex.unlock();
    lift_off_pos_left_mutex.unlock();
    lift_off_vel_left_mutex.unlock();
}



Eigen::Matrix<double, 5, 1> get_joint_torques(Eigen::Matrix<double, 3, 1> f, double theta1, double theta2, double theta3, double theta4, double theta5, double phi, double theta, double psi) {
    return (Eigen::Matrix<double, 5, 1>() << f(0, 0)*(0.41*sin(theta2)*sin(psi + theta1)*cos(theta3) + 0.4*sin(theta2)*sin(psi + theta1)*cos(theta3 + theta4) + 0.04*sin(theta2)*sin(psi + theta1)*cos(theta3 + theta4 + theta5) - 0.41*sin(theta3)*cos(psi + theta1) - 0.4*sin(theta3 + theta4)*cos(psi + theta1) - 0.04*sin(theta3 + theta4 + theta5)*cos(psi + theta1))*cos(theta) + f(1, 0)*((sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(0.41*sin(theta1)*sin(theta3) + 0.4*sin(theta1)*sin(theta3 + theta4) + 0.04*sin(theta1)*sin(theta3 + theta4 + theta5) + 0.41*sin(theta2)*cos(theta1)*cos(theta3) + 0.4*sin(theta2)*cos(theta1)*cos(theta3 + theta4) + 0.04*sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5)) + (sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(0.41*sin(theta1)*sin(theta2)*cos(theta3) + 0.4*sin(theta1)*sin(theta2)*cos(theta3 + theta4) + 0.04*sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) - 0.41*sin(theta3)*cos(theta1) - 0.4*sin(theta3 + theta4)*cos(theta1) - 0.04*sin(theta3 + theta4 + theta5)*cos(theta1))) + f(2, 0)*((sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(0.41*sin(theta1)*sin(theta2)*cos(theta3) + 0.4*sin(theta1)*sin(theta2)*cos(theta3 + theta4) + 0.04*sin(theta1)*sin(theta2)*cos(theta3 + theta4 + theta5) - 0.41*sin(theta3)*cos(theta1) - 0.4*sin(theta3 + theta4)*cos(theta1) - 0.04*sin(theta3 + theta4 + theta5)*cos(theta1)) - (sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(0.41*sin(theta1)*sin(theta3) + 0.4*sin(theta1)*sin(theta3 + theta4) + 0.04*sin(theta1)*sin(theta3 + theta4 + theta5) + 0.41*sin(theta2)*cos(theta1)*cos(theta3) + 0.4*sin(theta2)*cos(theta1)*cos(theta3 + theta4) + 0.04*sin(theta2)*cos(theta1)*cos(theta3 + theta4 + theta5))), f(0, 0)*((0.41*sin(theta2)*cos(theta3) + 0.4*sin(theta2)*cos(theta3 + theta4) + 0.04*sin(theta2)*cos(theta3 + theta4 + theta5))*sin(theta) + (0.41*cos(theta2)*cos(theta3) + 0.4*cos(theta2)*cos(theta3 + theta4) + 0.04*cos(theta2)*cos(theta3 + theta4 + theta5))*sin(psi)*sin(theta1)*cos(theta) - (0.41*cos(theta2)*cos(theta3) + 0.4*cos(theta2)*cos(theta3 + theta4) + 0.04*cos(theta2)*cos(theta3 + theta4 + theta5))*cos(psi)*cos(theta)*cos(theta1)) - f(1, 0)*(-(sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(0.41*cos(theta2)*cos(theta3) + 0.4*cos(theta2)*cos(theta3 + theta4) + 0.04*cos(theta2)*cos(theta3 + theta4 + theta5))*sin(theta1) + (sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(0.41*cos(theta2)*cos(theta3) + 0.4*cos(theta2)*cos(theta3 + theta4) + 0.04*cos(theta2)*cos(theta3 + theta4 + theta5))*cos(theta1) + (0.41*sin(theta2)*cos(theta3) + 0.4*sin(theta2)*cos(theta3 + theta4) + 0.04*sin(theta2)*cos(theta3 + theta4 + theta5))*sin(phi)*cos(theta)) - f(2, 0)*((sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(0.41*cos(theta2)*cos(theta3) + 0.4*cos(theta2)*cos(theta3 + theta4) + 0.04*cos(theta2)*cos(theta3 + theta4 + theta5))*cos(theta1) + (sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(0.41*cos(theta2)*cos(theta3) + 0.4*cos(theta2)*cos(theta3 + theta4) + 0.04*cos(theta2)*cos(theta3 + theta4 + theta5))*sin(theta1) - (0.41*sin(theta2)*cos(theta3) + 0.4*sin(theta2)*cos(theta3 + theta4) + 0.04*sin(theta2)*cos(theta3 + theta4 + theta5))*cos(phi)*cos(theta)), -f(0, 0)*(-0.41*sin(theta)*sin(theta3)*cos(theta2) - 0.4*sin(theta)*sin(theta3 + theta4)*cos(theta2) - 0.04*sin(theta)*sin(theta3 + theta4 + theta5)*cos(theta2) - 0.41*sin(theta2)*sin(theta3)*cos(theta)*cos(psi + theta1) - 0.4*sin(theta2)*sin(theta3 + theta4)*cos(theta)*cos(psi + theta1) - 0.04*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta)*cos(psi + theta1) + 0.41*sin(psi + theta1)*cos(theta)*cos(theta3) + 0.4*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4) + 0.04*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4 + theta5)) - f(1, 0)*((sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(0.41*sin(theta1)*sin(theta2)*sin(theta3) + 0.4*sin(theta1)*sin(theta2)*sin(theta3 + theta4) + 0.04*sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + 0.41*cos(theta1)*cos(theta3) + 0.4*cos(theta1)*cos(theta3 + theta4) + 0.04*cos(theta1)*cos(theta3 + theta4 + theta5)) + (sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(0.41*sin(theta1)*cos(theta3) + 0.4*sin(theta1)*cos(theta3 + theta4) + 0.04*sin(theta1)*cos(theta3 + theta4 + theta5) - 0.41*sin(theta2)*sin(theta3)*cos(theta1) - 0.4*sin(theta2)*sin(theta3 + theta4)*cos(theta1) - 0.04*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)) - (-0.41*sin(theta3) - 0.4*sin(theta3 + theta4) - 0.04*sin(theta3 + theta4 + theta5))*sin(phi)*cos(theta)*cos(theta2)) - f(2, 0)*((sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(0.41*sin(theta1)*cos(theta3) + 0.4*sin(theta1)*cos(theta3 + theta4) + 0.04*sin(theta1)*cos(theta3 + theta4 + theta5) - 0.41*sin(theta2)*sin(theta3)*cos(theta1) - 0.4*sin(theta2)*sin(theta3 + theta4)*cos(theta1) - 0.04*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)) - (sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(0.41*sin(theta1)*sin(theta2)*sin(theta3) + 0.4*sin(theta1)*sin(theta2)*sin(theta3 + theta4) + 0.04*sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + 0.41*cos(theta1)*cos(theta3) + 0.4*cos(theta1)*cos(theta3 + theta4) + 0.04*cos(theta1)*cos(theta3 + theta4 + theta5)) + (-0.41*sin(theta3) - 0.4*sin(theta3 + theta4) - 0.04*sin(theta3 + theta4 + theta5))*cos(phi)*cos(theta)*cos(theta2)), -f(0, 0)*(-0.4*sin(theta)*sin(theta3 + theta4)*cos(theta2) - 0.04*sin(theta)*sin(theta3 + theta4 + theta5)*cos(theta2) - 0.4*sin(theta2)*sin(theta3 + theta4)*cos(theta)*cos(psi + theta1) - 0.04*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta)*cos(psi + theta1) + 0.4*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4) + 0.04*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4 + theta5)) - f(1, 0)*((sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(0.4*sin(theta1)*sin(theta2)*sin(theta3 + theta4) + 0.04*sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + 0.4*cos(theta1)*cos(theta3 + theta4) + 0.04*cos(theta1)*cos(theta3 + theta4 + theta5)) + (sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi))*(0.4*sin(theta1)*cos(theta3 + theta4) + 0.04*sin(theta1)*cos(theta3 + theta4 + theta5) - 0.4*sin(theta2)*sin(theta3 + theta4)*cos(theta1) - 0.04*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)) - (-0.4*sin(theta3 + theta4) - 0.04*sin(theta3 + theta4 + theta5))*sin(phi)*cos(theta)*cos(theta2)) - f(2, 0)*((sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(0.4*sin(theta1)*cos(theta3 + theta4) + 0.04*sin(theta1)*cos(theta3 + theta4 + theta5) - 0.4*sin(theta2)*sin(theta3 + theta4)*cos(theta1) - 0.04*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)) - (sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(0.4*sin(theta1)*sin(theta2)*sin(theta3 + theta4) + 0.04*sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + 0.4*cos(theta1)*cos(theta3 + theta4) + 0.04*cos(theta1)*cos(theta3 + theta4 + theta5)) + (-0.4*sin(theta3 + theta4) - 0.04*sin(theta3 + theta4 + theta5))*cos(phi)*cos(theta)*cos(theta2)), -f(0, 0)*(-0.04*sin(theta)*sin(theta3 + theta4 + theta5)*cos(theta2) - 0.04*sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta)*cos(psi + theta1) + 0.04*sin(psi + theta1)*cos(theta)*cos(theta3 + theta4 + theta5)) - f(1, 0)*(-0.04*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1))*(sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(phi)) + 0.04*(sin(phi)*sin(psi)*sin(theta) - cos(phi)*cos(psi))*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5)) + 0.04*sin(phi)*sin(theta3 + theta4 + theta5)*cos(theta)*cos(theta2)) - f(2, 0)*(-0.04*(sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi))*(-sin(theta1)*cos(theta3 + theta4 + theta5) + sin(theta2)*sin(theta3 + theta4 + theta5)*cos(theta1)) - 0.04*(sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*(sin(theta1)*sin(theta2)*sin(theta3 + theta4 + theta5) + cos(theta1)*cos(theta3 + theta4 + theta5)) - 0.04*sin(theta3 + theta4 + theta5)*cos(phi)*cos(theta)*cos(theta2))).finished();
}

std::thread left_leg_state_thread; // Thread for updating left leg state based on gazebosim messages
std::thread left_leg_torque_thread; // Thread for updating matrices, calculating torque setpoint and sending torque setpoint to gazebosim
std::thread right_leg_torque_thread;
std::thread mpc_thread;

static const double state_update_interval = 1000.0; // Interval for fetching and parsing the leg state from gazebosim in microseconds
static const double torque_calculation_interval = 1000.0; // Interval for calculating and sending the torque setpoint to gazebosim in microseconds

void update_left_leg_state() {

    // High resolution clocks used for measuring execution time of loop iteration.

    high_resolution_clock::time_point start = high_resolution_clock::now();
    high_resolution_clock::time_point end = high_resolution_clock::now();

    double duration = 0.0f; // Duration double for storing execution duration

    struct timespec deadline; // timespec struct for storing time that execution thread should sleep for

    while(true) {
        
        start = high_resolution_clock::now();
        // Do stuff
        end = high_resolution_clock::now();

        // This timed loop approach calculates the execution time of the current iteration,
        // then calculates the remaining time for the loop to run at the desired frequency and waits this duration.

        duration = duration_cast<microseconds>(end - start).count();  
        long long remainder = (state_update_interval - duration) * 1e+03;
        deadline.tv_nsec = remainder;
        deadline.tv_sec = 0;
        clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
    }
}

void calculate_left_leg_torques() {

    // High resolution clocks used for measuring execution time of loop iteration.
    high_resolution_clock::time_point start = high_resolution_clock::now();
    high_resolution_clock::time_point end = high_resolution_clock::now();

    double duration = 0.0f; // Duration double for storing execution duration

    struct timespec deadline; // timespec struct for storing time that execution thread should sleep for

    Eigen::Matrix<double, 5, 1> q_temp; // Temporary leg angle vector used for thread-safe updating of matrices
    Eigen::Matrix<double, 5, 1> q_dot_temp; // Temporary leg angular velocity vector used for thread-safe updating of matrices

    long long iteration_counter = 0; // Iteration counter of the timed loop used for calculating current loop "time" and debugging
    double dt = torque_calculation_interval / 1000 / 1000; // Loop update interval in seconds for calculation current loop "time" based on iteration counter

    // Setting up UDP server socket...
    int sockfd;
    char buffer[udp_buffer_size];
    struct sockaddr_in servaddr, cliaddr; 
      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE);
    }
      
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
    
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
    servaddr.sin_port = htons(left_leg_torque_port); 
      
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE);
    } 
      
    int msg_length; 
    socklen_t len;

    ofstream data_file;
    data_file.open(".././plot_data/" + filename + "_left.csv");
    data_file << "t,"
                << "theta1,theta2,theta3,theta4,theta5,theta1_dot,theta2_dot,theta3_dot,theta4_dot,theta5_dot,"
                << "tau_1,tau_2,tau_3,tau_4,tau_5,"
                << "foot_pos_x,foot_pos_y,foot_pos_z,"
                << "foot_pos_x_desired,foot_pos_y_desired,foot_pos_z_desired"
                << "foot_vel_x,foot_vel_y,foot_vel_z,"
                << "foot_vel_x_desired,foot_vel_y_desired,foot_vel_z_desired" << std::endl; // Add header to csv file
    data_file.close();

    bool time_switch = false; // used for running a two-phase trajectory, otherwise obsolete

    while(true) {
        start = high_resolution_clock::now();

        double t = iteration_counter * dt;

        //Declaring angle and angular velocity variables for updating matrices
        
        double theta1 = 0;
        double theta2 = 0;
        double theta3 = 0;
        double theta4 = 0;
        double theta5 = 0;

        double theta1_dot = 0;
        double theta2_dot = 0;
        double theta3_dot = 0;
        double theta4_dot = 0;
        double theta5_dot = 0;

        u_mutex.lock();
        Eigen::Matrix<double, m, 1> u = u_t;
        u_mutex.unlock();

        x_mutex.lock();
        Eigen::Matrix<double, n, 1> x = x_t;
        x_mutex.unlock();

        double phi_com = x(0, 0);
        double theta_com = x(1, 0);
        double psi_com = x(2, 0);

        double pos_x_com = x(3, 0);
        double pos_y_com = x(4, 0);
        double pos_z_com = x(5, 0);

        double vel_x_com = x(9, 0);
        double vel_y_com = x(10, 0);
        double vel_z_com = x(11, 0);

        msg_length = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, ( struct sockaddr *) &cliaddr, &len); // Receive message over UDP containing full leg state
        buffer[msg_length] = '\0'; // Add string ending delimiter to end of string (n is length of message)

        std::string raw_state(buffer); // Create string from buffer char array to split

        std::vector<std::string> state = split_string(raw_state, '|'); // Split raw state message by message delimiter to parse individual elements

        if(static_cast<int>(state.size()) >= 9) { // Check if message is complete. TODO: Add unique character to end of message for extra check

            // Convert individual string elements to float
            theta1 = atof(state[0].c_str());
            theta2 = atof(state[1].c_str());
            theta3 = atof(state[2].c_str());
            theta4 = atof(state[3].c_str());
            theta5 = atof(state[4].c_str());

            theta1_dot = atof(state[5].c_str());
            theta2_dot = atof(state[6].c_str());
            theta3_dot = atof(state[7].c_str());
            theta4_dot = atof(state[8].c_str());
            theta5_dot = atof(state[9].c_str());

            // Update states based on parsed angles

            q_temp << theta1, theta2, theta3, theta4, theta5;
            q_dot_temp << theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot;
        }

        // If swing, leg trajectory should be followed, if not, foot is in contact with the ground and MPC forces should be converted into torques and applied
        if(swing_left) {
            
            // pos_desired_left_leg << 0, 0, 0.1*sin(2*t) - 0.9, 0, 0;

            // vel_desired_left_leg << 0, 0, 0.2*cos(2*t), 0, 0;

            // accel_desired_left_leg << 0, 0, -0.4*sin(2*t);

            // if(iteration_counter % 1500 == 0) {
            //     time_switch = !time_switch;
            //     iteration_counter = 0;
            // }

            double x_pos_t = 0; // Current desired cartesian x position
            double x_vel_t = 0; // Current desired cartesian x velocity
            double x_accel_t = 0; // Current desired cartesian x acceleration

            double y_pos_t = 0; // Current desired cartesian y position
            double y_vel_t = 0; // Current desired cartesian y velocity
            double y_accel_t = 0; // Current desired cartesian y acceleration

            double z_pos_t = 0; // Current desired cartesian z position
            double z_vel_t = 0; // Current desired cartesian z velocity
            double z_accel_t = 0; // Current desired cartesain z acceleration

            double phi_t_left_leg = 0; // Current desired roll orientation for the foot described as an Euler Angle
            double phi_dot_t_left_leg = 0; // Current desired roll angular velocity for the foot described as an Euler Angle

            double psi_t_left_leg = 0; // Current desired yaw orientation for the foot described as an Euler Angle
            double psi_dot_t_left_leg = 0; // Current desired yaw angular velocity for the foot described as an Euler Angle
            
            t_stance_remainder_left_mutex.lock();
            // Get setpoint from trajectory
            
            t_stance_remainder_left_mutex.unlock();

            foot_trajectory_left_mutex.lock();

            pos_desired_left_leg << foot_trajectory_left(0, 0) + hip_offset_left_leg, foot_trajectory_left(0, 2), foot_trajectory_left(0, 4), 0, 0;
            vel_desired_left_leg << foot_trajectory_left(0, 1), foot_trajectory_left(0, 3), foot_trajectory_left(0, 5), 0, 0;

            foot_trajectory_left_mutex.unlock();

            std::cout << "pos_desired_left_leg: " << pos_desired_left_leg(0, 0) << "," << pos_desired_left_leg(1, 0) << pos_desired_left_leg(2, 0) << std::endl;
            std::cout << "left_foot_pos_desired_world: " << left_foot_pos_desired_world(0, 0) << "," << left_foot_pos_desired_world(1, 0) << "," << left_foot_pos_desired_world(2, 0) << std::endl;

            // Eigen::Matrix<double, 3,1> pos_desired_left_leg_cartesian = (H_world_hip * (Eigen::Matrix<double, 4, 1>() << left_foot_pos_world, 1).finished()).block<3,1>(0, 0);

            // pos_desired_left_leg << pos_desired_left_leg_cartesian, 0, 0;

            // std::cout << "pos_desired_left_leg: " << pos_desired_left_leg(0, 0) << "," << pos_desired_left_leg(1, 0) << "," << pos_desired_left_leg(2, 0) << std::endl;
            // std::cout << "left_foot_pos_world: " << left_foot_pos_world(0, 0) << "," << left_foot_pos_world(1, 0) << "," << left_foot_pos_world(2, 0) << std::endl;
            // std::cout << "pos_z_com:" << pos_z_com << std::endl;

            // Updating desired trajectory

            // x_pos_t = 0;
            // x_vel_t = 0;
            // x_accel_t = 0;
            
            // double omega = 8.0; // Frequency for sinusoidal Trajectory in rad/s

            // // X:

            // //x_pos_t = 0.200000000000000011102L*cosl(2*t);
            // x_pos_t = 0.1 * sin(omega*t);
            // x_vel_t = 0.1*omega*cos(omega*t);
            // x_accel_t = -0.1*omega*omega*sin(omega*t);

            // // Y:

            // y_pos_t = 0.200000000000000011102L*cosl(omega*t);
            // y_vel_t = -0.200000000000000011102L*omega*sinl(omega*t);
            // y_accel_t = -0.200000000000000011102L*powl(omega, 2)*cosl(omega*t);

            // // Z:

            // z_pos_t = 0.100000000000000005551L*sinl(omega*t) - 0.800000000000000044409L;
            // z_vel_t = 0.100000000000000005551L*omega*cosl(omega*t);
            // z_accel_t = -0.100000000000000005551L*powl(omega, 2)*sinl(omega*t);

            // pos_desired_left_leg << x_pos_t, y_pos_t, z_pos_t, phi_t_left_leg, psi_t_left_leg;
            // vel_desired_left_leg << x_vel_t, y_vel_t, z_vel_t, psi_t_left_leg, psi_dot_t_left_leg;
            // accel_desired_left_leg << x_accel_t, y_accel_t, z_accel_t;

            // std::cout << "pos: " << pos_desired_left_leg(0) << ", " << pos_desired_left_leg(1) << ", " << pos_desired_left_leg(2) << ", vel: " << vel_desired_left_leg(0)
            //             << ", " << vel_desired_left_leg(1) << ", " << vel_desired_left_leg(2) 
            //             << ", accel: " << accel_desired_left_leg(0) << ", " << accel_desired_left_leg(1) << ", " << accel_desired_left_leg(2) << std::endl;

            //TODO: Maybe rework to only use q and q_dot

            // Update matrices based on received angles and angular velocities

            update_orientation_left_leg(theta1, theta2, theta3, theta4, theta5);

            update_B_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

            update_J_foot_left_leg(theta1, theta2, theta3, theta4, theta5);

            update_J_foot_combined_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

            update_J_foot_dot_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

            //Set singular Jacobians to zero, filter and constrain elements

            for(int i = 0, nCols = J_foot_left_leg.cols(), nRows = J_foot_left_leg.rows(); i < nCols; ++i) {
                for(int j = 0; i < nRows; ++i) {
                    if(isnan(J_foot_left_leg(j, i)) || isinf(J_foot_left_leg(j, i))) {
                        J_foot_left_leg(j, i) = 0;
                    }
                }
            }

            for(int i = 0, nCols = J_foot_dot_left_leg.cols(), nRows = J_foot_dot_left_leg.rows(); i < nCols; ++i) {
                for(int j = 0; i < nRows; ++i) {
                    if(isnan(J_foot_dot_left_leg(j, i)) || isinf(J_foot_dot_left_leg(j, i))) {
                        J_foot_dot_left_leg(j, i) = 0;
                    }
                }
            }

            for(int i = 0, nCols = J_foot_combined_left_leg.cols(), nRows = J_foot_combined_left_leg.rows(); i < nCols; ++i) {
                for(int j = 0; i < nRows; ++i) {
                    if(isnan(J_foot_combined_left_leg(j, i)) || isinf(J_foot_combined_left_leg(j, i))) {
                        J_foot_combined_left_leg(j, i) = 0;
                    }
                }
            }

            update_G_left_leg(theta1, theta2, theta3, theta4, theta5);

            update_C_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

            update_Lambda_left_leg();
            
            for(int i = 0, nCols = Lambda_left_leg.cols(), nRows = Lambda_left_leg.rows(); i < nCols; ++i) {
                for(int j = 0; i < nRows; ++i) {
                    if(isnan(Lambda_left_leg(j, i)) || isinf(Lambda_left_leg(j, i))) {
                        Lambda_left_leg(j, i) = 0;
                    }
                }
            }

            update_tau_ff_left_leg(q_dot_temp);

            update_Kp_left_leg();
            update_Kd_left_leg();

            update_foot_pos_left_leg(theta1, theta2, theta3, theta4, theta5);

            //std::cout << "Foot Position: " << foot_pos_left_leg(0) << ", " << foot_pos_left_leg(1) << ", " << foot_pos_left_leg(2) << std::endl;

            //std::cout << '\r' << std::setw(2) << std::setfill('0') << h << ':' << std::setw(2) << m << ':' << std::setw(2) << s << std::flush;

            update_foot_vel_left_leg(q_dot_temp);

            update_tau_setpoint_left_leg();

            for(int i = 0; i < 5; ++i) { // Loop through each torque setpoint vector element
                constrain(tau_setpoint_left_leg(i), lower_torque_limit, upper_torque_limit); // constrain element based on global torque limits
            }
            constrain(tau_setpoint_left_leg(4), -5, 5);

            if(iteration_counter % 1 == 0) {
                /*
                    << "t,"
                    << "theta1,theta2,theta3,theta4,theta5,theta1_dot,theta2_dot,theta3_dot,theta4_dot,theta5_dot,"
                    << "tau_1,tau_2,tau_3,tau_4,tau_5,"
                    << "foot_pos_x,foot_pos_y,foot_pos_z,"
                    << "foot_pos_x_desired,foot_pos_y_desired,foot_pos_z_desired"
                    << "foot_vel_x,foot_vel_y,foot_vel_z,"
                    << "foot_vel_x_desired,foot_vel_y_desired,foot_vel_z_desired" << std::endl;
                */
                
                ofstream data_file;
                data_file.open(".././plot_data/" + filename + "_left.csv", ios::app); // Open csv file in append mode
                data_file << t // Write plot values to csv file
                            << "," << theta1 << "," << theta2 << "," << theta3 << "," << theta4 << "," << theta5
                            << "," << theta1_dot << "," << theta2_dot << "," << theta3_dot << "," << theta4_dot << "," << theta5_dot
                            << "," << tau_setpoint_left_leg(0) << "," << tau_setpoint_left_leg(1) << "," << tau_setpoint_left_leg(2) << "," << tau_setpoint_left_leg(3) << "," << tau_setpoint_left_leg(4)
                            << "," << foot_pos_left_leg(0) << "," << foot_pos_left_leg(1) << "," << foot_pos_left_leg(2)
                            << "," << x_pos_t << "," << y_pos_t << "," << z_pos_t
                            << "," << foot_vel_left_leg(0) << "," << foot_vel_left_leg(1) << "," << foot_vel_left_leg(2)
                            << "," << x_vel_t << "," << y_vel_t << "," << z_vel_t << std::endl;
                    
                data_file.close(); // Close csv file again. This way thread abort should (almost) never leave file open.
            }

            stringstream s;
            s << tau_setpoint_left_leg(0) << "|" << tau_setpoint_left_leg(1) << "|" << tau_setpoint_left_leg(2) << "|" << tau_setpoint_left_leg(3) << "|" << tau_setpoint_left_leg(4); // Write torque setpoints to stringstream
            sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), 
                MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len); // Send the torque setpoint string to the simulation
        }
        else {
            Eigen::Matrix<double, 5, 1> tau_setpoint = Eigen::ArrayXXd::Zero(5, 1); // get_joint_torques(u.block<3,1>(0, 0), theta1, theta2, theta3, theta4, theta5, x(0, 0), x(1, 0), x(2, 0));
            
            for(int i = 0; i < 5; ++i) {
                constrain(tau_setpoint(i ,0), lower_torque_limit, upper_torque_limit);
            }

            stringstream s;
            s << tau_setpoint(0, 0) << "|" << tau_setpoint(1, 0) << "|" << tau_setpoint(2, 0) << "|" << tau_setpoint(3, 0) << "|" << tau_setpoint(4, 0);
            sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), 
                MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len); // Send the torque setpoint string to the simulation
            
            std::cout << "tau_setpoint_left_leg: " << s.str() << std::endl;
        }


        iteration_counter++; // Increment iteration counter
        t_stance_remainder_left_mutex.lock();
        t_stance_remainder_left -= 1/torque_calculation_interval;
        constrain(t_stance_remainder_left, 0, 10000);

        std::cout << "t_stance_remainder_left: " << t_stance_remainder_left << std::endl;
        t_stance_remainder_left_mutex.unlock();

        end = high_resolution_clock::now();

        // This timed loop approach calculates the execution time of the current iteration,
        // then calculates the remaining time for the loop to run at the desired frequency and waits this duration.

        duration = duration_cast<microseconds>(end - start).count();
        // std::cout << "Loop duration: " << duration << "µS, iteration_counter: " << iteration_counter - 1 << std::endl;
        long long remainder = (torque_calculation_interval - duration) * 1e+03;
        deadline.tv_nsec = remainder;
        deadline.tv_sec = 0;
        clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
    }
}

void calculate_right_leg_torques() {

    // High resolution clocks used for measuring execution time of loop iteration.
    high_resolution_clock::time_point start = high_resolution_clock::now();
    high_resolution_clock::time_point end = high_resolution_clock::now();

    double duration = 0.0f; // Duration double for storing execution duration

    struct timespec deadline; // timespec struct for storing time that execution thread should sleep for

    Eigen::Matrix<double, 5, 1> q_temp; // Temporary leg angle vector used for thread-safe updating of matrices
    Eigen::Matrix<double, 5, 1> q_dot_temp; // Temporary leg angular velocity vector used for thread-safe updating of matrices

    long long iteration_counter = 0; // Iteration counter of the timed loop used for calculating current loop "time" and debugging
    double dt = torque_calculation_interval / 1000 / 1000; // Loop update interval in seconds for calculation current loop "time" based on iteration counter

    // Setting up UDP server socket...
    int sockfd;
    char buffer[udp_buffer_size];
    struct sockaddr_in servaddr, cliaddr; 
      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE);
    }
      
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
    
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); 
    servaddr.sin_port = htons(right_leg_torque_port); 
      
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE);
    } 
      
    int msg_length; 
    socklen_t len;

    ofstream data_file;
    data_file.open(".././plot_data/" + filename + "_right.csv");
    data_file << "t,"
                << "theta1,theta2,theta3,theta4,theta5,theta1_dot,theta2_dot,theta3_dot,theta4_dot,theta5_dot,"
                << "tau_1,tau_2,tau_3,tau_4,tau_5,"
                << "foot_pos_x,foot_pos_y,foot_pos_z,"
                << "foot_pos_x_desired,foot_pos_y_desired,foot_pos_z_desired"
                << "foot_vel_x,foot_vel_y,foot_vel_z,"
                << "foot_vel_x_desired,foot_vel_y_desired,foot_vel_z_desired" << std::endl; // Add header to csv file
    data_file.close();

    bool time_switch = false; // used for running a two-phase trajectory, otherwise obsolete

    double theta1 = 0;
    double theta2 = 0;
    double theta3 = 0;
    double theta4 = 0;
    double theta5 = 0;

    double theta1_dot = 0;
    double theta2_dot = 0;
    double theta3_dot = 0;
    double theta4_dot = 0;
    double theta5_dot = 0;

    while(true) {
        start = high_resolution_clock::now();
        
        double t = iteration_counter * dt;

        //Declaring angle and angular velocity variables for updating matrices

        u_mutex.lock();
        Eigen::Matrix<double, m, 1> u = u_t;
        u_mutex.unlock();
        
        x_mutex.lock();
        Eigen::Matrix<double, n, 1> x = x_t;
        x_mutex.unlock();

        double phi_com = x(0, 0);
        double theta_com = x(1, 0);
        double psi_com = x(2, 0);

        double pos_x_com = x(3, 0);
        double pos_y_com = x(4, 0);
        double pos_z_com = x(5, 0);

        msg_length= recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, ( struct sockaddr *) &cliaddr, &len); // Receive message over UDP containing full leg state
        buffer[msg_length] = '\0'; // Add string ending delimiter to end of string (n is length of message)

        std::string raw_state(buffer); // Create string from buffer char array to split

        std::vector<std::string> state = split_string(raw_state, '|'); // Split raw state message by message delimiter to parse individual elements

        if(static_cast<int>(state.size()) >= 9) { // Check if message is complete. TODO: Add unique character to end of message for extra check

            // Convert individual string elements to float
            theta1 = atof(state[0].c_str());
            theta2 = atof(state[1].c_str());
            theta3 = atof(state[2].c_str());
            theta4 = atof(state[3].c_str());
            theta5 = atof(state[4].c_str());

            theta1_dot = atof(state[5].c_str());
            theta2_dot = atof(state[6].c_str());
            theta3_dot = atof(state[7].c_str());
            theta4_dot = atof(state[8].c_str());
            theta5_dot = atof(state[9].c_str());

            // Update states based on parsed angles

            q_temp << theta1, theta2, theta3, theta4, theta5;
            q_dot_temp << theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot;
        }

        // If swing, leg trajectory should be followed, if not, foot is in contact with the ground and MPC forces should be converted into torques and applied
        if(swing_right) {
            
            // pos_desired_right_leg << 0, 0, 0.1*sin(2*t) - 0.9, 0, 0;

            // vel_desired_right_leg << 0, 0, 0.2*cos(2*t), 0, 0;

            // accel_desired_right_leg << 0, 0, -0.4*sin(2*t);

            // if(iteration_counter % 1500 == 0) {
            //     time_switch = !time_switch;
            //     iteration_counter = 0;
            // }

            double x_pos_t = 0; // Current desired cartesian x position
            double x_vel_t = 0; // Current desired cartesian x velocity
            double x_accel_t = 0; // Current desired cartesian x acceleration

            double y_pos_t = 0; // Current desired cartesian y position
            double y_vel_t = 0; // Current desired cartesian y velocity
            double y_accel_t = 0; // Current desired cartesian y acceleration

            double z_pos_t = 0; // Current desired cartesian z position
            double z_vel_t = 0; // Current desired cartesian z velocity
            double z_accel_t = 0; // Current desired cartesain z acceleration

            double phi_t_left_leg = 0; // Current desired roll orientation for the foot described as an Euler Angle
            double phi_dot_t_left_leg = 0; // Current desired roll angular velocity for the foot described as an Euler Angle

            double psi_t_left_leg = 0; // Current desired yaw orientation for the foot described as an Euler Angle
            double psi_dot_t_left_leg = 0; // Current desired yaw angular velocity for the foot described as an Euler Angle

            // Updating desired trajectory

            Eigen::Matrix<double, 4, 4> H_world_hip = (Eigen::Matrix<double, 4, 4>() << cos(psi_com)*cos(theta_com), sin(psi_com)*cos(theta_com), -sin(theta_com), -pos_y_com*sin(psi_com)*cos(theta_com) + pos_z_com*sin(theta_com) - (hip_offset_right_leg + pos_x_com)*cos(psi_com)*cos(theta_com),
                                                        sin(phi_com)*sin(theta_com)*cos(psi_com) - sin(psi_com)*cos(phi_com), sin(phi_com)*sin(psi_com)*sin(theta_com) + cos(phi_com)*cos(psi_com), sin(phi_com)*cos(theta_com), -pos_y_com*sin(phi_com)*sin(psi_com)*sin(theta_com) - pos_y_com*cos(phi_com)*cos(psi_com) - pos_z_com*sin(phi_com)*cos(theta_com) - (hip_offset_right_leg + pos_x_com)*sin(phi_com)*sin(theta_com)*cos(psi_com) + (hip_offset_right_leg + pos_x_com)*sin(psi_com)*cos(phi_com), 
                                                        sin(phi_com)*sin(psi_com) + sin(theta_com)*cos(phi_com)*cos(psi_com), -sin(phi_com)*cos(psi_com) + sin(psi_com)*sin(theta_com)*cos(phi_com), cos(phi_com)*cos(theta_com), pos_y_com*sin(phi_com)*cos(psi_com) - pos_y_com*sin(psi_com)*sin(theta_com)*cos(phi_com) - pos_z_com*cos(phi_com)*cos(theta_com) - (hip_offset_right_leg + pos_x_com)*sin(phi_com)*sin(psi_com) - (hip_offset_right_leg + pos_x_com)*sin(theta_com)*cos(phi_com)*cos(psi_com),
                                                        0, 0, 0, 1).finished();
            right_foot_pos_world_mutex.lock();
            Eigen::Matrix<double, 3,1> pos_desired_right_leg_cartesian = (H_world_hip * (Eigen::Matrix<double, 4, 1>() << right_foot_pos_world, 1).finished()).block<3,1>(0, 0);
            right_foot_pos_world_mutex.unlock();
            pos_desired_right_leg << pos_desired_right_leg_cartesian, 0, 0;

            // x_pos_t = 0;
            // x_vel_t = 0;
            // x_accel_t = 0;
            
            // double omega = 8.0; // Frequency for sinusoidal Trajectory in rad/s

            // // X:

            // //x_pos_t = 0.200000000000000011102L*cosl(2*t);
            // x_pos_t = 0.1 * sin(omega*t);
            // x_vel_t = 0.1*omega*cos(omega*t);
            // x_accel_t = -0.1*omega*omega*sin(omega*t);

            // // Y:

            // y_pos_t = 0.200000000000000011102L*cosl(omega*t);
            // y_vel_t = -0.200000000000000011102L*omega*sinl(omega*t);
            // y_accel_t = -0.200000000000000011102L*powl(omega, 2)*cosl(omega*t);

            // // Z:

            // z_pos_t = 0.100000000000000005551L*sinl(omega*t) - 0.800000000000000044409L;
            // z_vel_t = 0.100000000000000005551L*omega*cosl(omega*t);
            // z_accel_t = -0.100000000000000005551L*powl(omega, 2)*sinl(omega*t);

            // pos_desired_right_leg << x_pos_t, y_pos_t, z_pos_t, phi_t_left_leg, psi_t_left_leg;
            // vel_desired_right_leg << x_vel_t, y_vel_t, z_vel_t, psi_t_left_leg, psi_dot_t_left_leg;
            // accel_desired_right_leg << x_accel_t, y_accel_t, z_accel_t;

            // std::cout << "pos: " << pos_desired_right_leg(0) << ", " << pos_desired_right_leg(1) << ", " << pos_desired_right_leg(2) << ", vel: " << vel_desired_right_leg(0)
            //             << ", " << vel_desired_right_leg(1) << ", " << vel_desired_right_leg(2) 
            //             << ", accel: " << accel_desired_right_leg(0) << ", " << accel_desired_right_leg(1) << ", " << accel_desired_right_leg(2) << std::endl;

            //TODO: Maybe rework to only use q and q_dot

            // Update matrices based on received angles and angular velocities

            update_orientation_right_leg(theta1, theta2, theta3, theta4, theta5);

            update_B_right_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

            update_J_foot_right_leg(theta1, theta2, theta3, theta4, theta5);

            update_J_foot_combined_right_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

            update_J_foot_dot_right_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

            //Set singular Jacobians to zero, filter and constrain elements

            for(int i = 0, nCols = J_foot_right_leg.cols(), nRows = J_foot_right_leg.rows(); i < nCols; ++i) {
                for(int j = 0; i < nRows; ++i) {
                    if(isnan(J_foot_right_leg(j, i)) || isinf(J_foot_right_leg(j, i))) {
                        J_foot_right_leg(j, i) = 0;
                    }
                }
            }

            for(int i = 0, nCols = J_foot_dot_right_leg.cols(), nRows = J_foot_dot_right_leg.rows(); i < nCols; ++i) {
                for(int j = 0; i < nRows; ++i) {
                    if(isnan(J_foot_dot_right_leg(j, i)) || isinf(J_foot_dot_right_leg(j, i))) {
                        J_foot_dot_right_leg(j, i) = 0;
                    }
                }
            }

            for(int i = 0, nCols = J_foot_combined_right_leg.cols(), nRows = J_foot_combined_right_leg.rows(); i < nCols; ++i) {
                for(int j = 0; i < nRows; ++i) {
                    if(isnan(J_foot_combined_right_leg(j, i)) || isinf(J_foot_combined_right_leg(j, i))) {
                        J_foot_combined_right_leg(j, i) = 0;
                    }
                }
            }

            update_G_right_leg(theta1, theta2, theta3, theta4, theta5);

            update_C_right_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

            update_Lambda_right_leg();

            for(int i = 0, nCols = Lambda_right_leg.cols(), nRows = Lambda_right_leg.rows(); i < nCols; ++i) {
                for(int j = 0; i < nRows; ++i) {
                    if(isnan(Lambda_right_leg(j, i)) || isinf(Lambda_right_leg(j, i))) {
                        Lambda_right_leg(j, i) = 0;
                    }
                }
            }

            update_tau_ff_right_leg(q_dot_temp);

            update_Kp_right_leg();
            update_Kd_right_leg();

            update_foot_pos_right_leg(theta1, theta2, theta3, theta4, theta5);

            //std::cout << "Foot Position: " << foot_pos_right_leg(0) << ", " << foot_pos_right_leg(1) << ", " << foot_pos_right_leg(2) << std::endl;

            //std::cout << '\r' << std::setw(2) << std::setfill('0') << h << ':' << std::setw(2) << m << ':' << std::setw(2) << s << std::flush;

            update_foot_vel_right_leg(q_dot_temp);

            update_tau_setpoint_right_leg();

            for(int i = 0; i < 5; ++i) { // Loop through each torque setpoint vector element
                constrain(tau_setpoint_right_leg(i), lower_torque_limit, upper_torque_limit); // constrain element based on global torque limits
            }
            constrain(tau_setpoint_right_leg(4), -5, 5);

            if(iteration_counter % 1 == 0) {
                /*
                    << "t,"
                    << "theta1,theta2,theta3,theta4,theta5,theta1_dot,theta2_dot,theta3_dot,theta4_dot,theta5_dot,"
                    << "tau_1,tau_2,tau_3,tau_4,tau_5,"
                    << "foot_pos_x,foot_pos_y,foot_pos_z,"
                    << "foot_pos_x_desired,foot_pos_y_desired,foot_pos_z_desired"
                    << "foot_vel_x,foot_vel_y,foot_vel_z,"
                    << "foot_vel_x_desired,foot_vel_y_desired,foot_vel_z_desired" << std::endl;
                */
                
                ofstream data_file;
                data_file.open(".././plot_data/" + filename + "_right.csv", ios::app); // Open csv file in append mode
                data_file << t // Write plot values to csv file
                            << "," << theta1 << "," << theta2 << "," << theta3 << "," << theta4 << "," << theta5
                            << "," << theta1_dot << "," << theta2_dot << "," << theta3_dot << "," << theta4_dot << "," << theta5_dot
                            << "," << tau_setpoint_right_leg(0) << "," << tau_setpoint_right_leg(1) << "," << tau_setpoint_right_leg(2) << "," << tau_setpoint_right_leg(3) << "," << tau_setpoint_right_leg(4)
                            << "," << foot_pos_right_leg(0) << "," << foot_pos_right_leg(1) << "," << foot_pos_right_leg(2)
                            << "," << x_pos_t << "," << y_pos_t << "," << z_pos_t
                            << "," << foot_vel_right_leg(0) << "," << foot_vel_right_leg(1) << "," << foot_vel_right_leg(2)
                            << "," << x_vel_t << "," << y_vel_t << "," << z_vel_t << std::endl;
                    
                data_file.close(); // Close csv file again. This way thread abort should (almost) never leave file open.
            }

            stringstream s;
            s << tau_setpoint_right_leg(0) << "|" << tau_setpoint_right_leg(1) << "|" << tau_setpoint_right_leg(2) << "|" << tau_setpoint_right_leg(3) << "|" << tau_setpoint_right_leg(4); // Write torque setpoints to stringstream
            sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), 
                MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len); // Send the torque setpoint string to the simulation
        }
        else {
            Eigen::Matrix<double, 5, 1> tau_setpoint =  Eigen::ArrayXXd::Zero(5, 1); //get_joint_torques(u.block<3,1>(3, 0), theta1, theta2, theta3, theta4, theta5, x(0, 0), x(1, 0), x(2, 0));
            
            for(int i = 0; i < 5; ++i) {
                constrain(tau_setpoint(i ,0), lower_torque_limit, upper_torque_limit);
            }

            stringstream s;
            s << tau_setpoint(0, 0) << "|" << tau_setpoint(1, 0) << "|" << tau_setpoint(2, 0) << "|" << tau_setpoint(3, 0) << "|" << tau_setpoint(4, 0);
            sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), 
                MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len); // Send the torque setpoint string to the simulation
            
            std::cout << "tau_setpoint_right_leg: " << s.str() << std::endl;
        }

        iteration_counter++; // Increment iteration counter
        t_stance_remainder_right_mutex.lock();
        t_stance_remainder_right -= 1/torque_calculation_interval;
        constrain(t_stance_remainder_right, 0, 10000);
        std::cout << "t_stance_remainder_right: " << t_stance_remainder_right << std::endl;
        t_stance_remainder_right_mutex.unlock();

        end = high_resolution_clock::now();

        // This timed loop approach calculates the execution time of the current iteration,
        // then calculates the remaining time for the loop to run at the desired frequency and waits this duration.

        duration = duration_cast<microseconds>(end - start).count();
        // std::cout << "Loop duration: " << duration << "µS, iteration_counter: " << iteration_counter - 1 << std::endl;
        long long remainder = (torque_calculation_interval - duration) * 1e+03;
        deadline.tv_nsec = remainder;
        deadline.tv_sec = 0;
        clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
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

static const Eigen::MatrixXd I_body = (Eigen::Matrix<double, 3, 3>() << 0.15, 0.0, 0.0,
                                                                        0.0, 0.62288, 0.0,
                                                                        0.0, 0.0, 0.68).finished(); // Inertia around CoM, meaning in Body Frame

static const double Ixx = I_body(0, 0);
static const double Ixy = I_body(0, 1);
static const double Ixz = I_body(0, 2);

static const double Iyx = I_body(1, 0);
static const double Iyy = I_body(1, 1);
static const double Iyz = I_body(1, 2);

static const double Izx = I_body(2, 0);
static const double Izy = I_body(2, 1);
static const double Izz = I_body(2, 2);

// Discretize the set of continuous state space matrices with a given timestep length based on https://en.wikipedia.org/wiki/Discretization#Discretization_of_linear_state_space_models
void discretize_state_space_matrices(Eigen::Matrix<double, n, n> &A_c_temp, Eigen::Matrix<double, n, m> &B_c_temp, const double &dt, Eigen::Matrix<double, n, n> &A_d_temp, Eigen::Matrix<double, n, m> &B_d_temp) {
    Eigen::Matrix<double, n+m, n+m> A_B = Eigen::ArrayXXd::Zero(n+m, n+m);
    // See linked wikipedia article
    A_B.block<n, n>(0, 0) = A_c_temp;
    A_B.block<n, m>(0, n) = B_c_temp;
    Eigen::MatrixXd e_A_B = (A_B * dt).exp();

    A_d_temp = e_A_B.block<n, n>(0, 0);
    B_d_temp = e_A_B.block<n, m>(0, n);
}

// Step the discretized model. This calls discretize_state_space_matrices and applies control u to the model with current state x in the form of x_{t+1} = A * x_t + B * u_t
Eigen::Matrix<double, n, 1> step_discrete_model(Eigen::Matrix<double, n, 1> x, Eigen::Matrix<double, m, 1> u, double r_x_left, double r_x_right, double r_y_left, double r_y_right, double r_z_left, double r_z_right) {

    double phi_t = x(0, 0);
    double theta_t = x(1, 0);
    double psi_t = x(2, 0);

    Eigen::Matrix<double, 3, 3> I_world = Eigen::ArrayXXd::Zero(3, 3);

    I_world << (sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixz*cos(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izz*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))) + (sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))*(Ixy*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))) + (Ixx*cos(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izx*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)))*cos(psi_t)*cos(theta_t), (-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixz*cos(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izz*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))) + (sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))) + (Ixx*cos(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izx*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)))*sin(psi_t)*cos(theta_t), (Ixy*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)))*sin(phi_t)*cos(theta_t) - (Ixx*cos(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izx*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)))*sin(theta_t) + (Ixz*cos(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izz*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)))*cos(phi_t)*cos(theta_t),
                (sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixz*sin(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izz*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))) + (sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))*(Ixy*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Ixy*sin(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))) + (Ixx*sin(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izx*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*cos(psi_t)*cos(theta_t), (-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixz*sin(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izz*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))) + (sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Ixy*sin(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))) + (Ixx*sin(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izx*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*sin(psi_t)*cos(theta_t), (Ixy*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Ixy*sin(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)))*sin(phi_t)*cos(theta_t) - (Ixx*sin(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izx*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*sin(theta_t) + (Ixz*sin(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izz*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*cos(phi_t)*cos(theta_t),
                (sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))*(-Ixz*sin(theta_t) + Iyz*sin(phi_t)*cos(theta_t) + Izz*cos(phi_t)*cos(theta_t)) + (sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))*(-Ixy*sin(theta_t) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*sin(phi_t)*cos(theta_t)) + (-Ixx*sin(theta_t) + Iyx*sin(phi_t)*cos(theta_t) + Izx*cos(phi_t)*cos(theta_t))*cos(psi_t)*cos(theta_t), (-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(-Ixz*sin(theta_t) + Iyz*sin(phi_t)*cos(theta_t) + Izz*cos(phi_t)*cos(theta_t)) + (sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(-Ixy*sin(theta_t) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*sin(phi_t)*cos(theta_t)) + (-Ixx*sin(theta_t) + Iyx*sin(phi_t)*cos(theta_t) + Izx*cos(phi_t)*cos(theta_t))*sin(psi_t)*cos(theta_t), -(-Ixx*sin(theta_t) + Iyx*sin(phi_t)*cos(theta_t) + Izx*cos(phi_t)*cos(theta_t))*sin(theta_t) + (-Ixy*sin(theta_t) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*sin(phi_t)*cos(theta_t))*sin(phi_t)*cos(theta_t) + (-Ixz*sin(theta_t) + Iyz*sin(phi_t)*cos(theta_t) + Izz*cos(phi_t)*cos(theta_t))*cos(phi_t)*cos(theta_t);

    Eigen::Matrix<double, 3, 3> r_left_skew_symmetric_test = Eigen::ArrayXXd::Zero(3, 3);
    Eigen::Matrix<double, 3, 3> r_right_skew_symmetric_test = Eigen::ArrayXXd::Zero(3, 3);

    r_left_skew_symmetric_test << 0, -r_z_left, r_y_left,
                                r_z_left, 0, -r_x_left,
                                -r_y_left, r_x_left, 0;
            
    r_right_skew_symmetric_test << 0, -r_z_right, r_y_right,
                                r_z_right, 0, -r_x_right,
                                -r_y_right, r_x_right, 0;

    Eigen::Matrix<double, n, n> A_c = Eigen::ArrayXXd::Zero(n, n);
    Eigen::Matrix<double, n, m> B_c = Eigen::ArrayXXd::Zero(n, m);

    A_c << 0, 0, 0, 0, 0, 0, cos(psi_t) / cos(theta_t), sin(psi_t) / cos(theta_t), 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, -sin(psi_t), cos(psi_t), 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, cos(psi_t) * tan(theta_t), sin(psi_t)*tan(theta_t), 1, 0, 0, 0, 0,
                    
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                    
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                    
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    B_c << 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,   
                    I_world.inverse() * r_left_skew_symmetric_test, I_world.inverse() * r_right_skew_symmetric_test,
                    1/m_value, 0, 0, 1/m_value, 0, 0,
                    0, 1/m_value, 0, 0, 1/m_value, 0,
                    0, 0, 1/m_value, 0, 0, 1/m_value,
                    0, 0, 0, 0, 0, 0;

    Eigen::Matrix<double, n, n> A_d = Eigen::ArrayXXd::Zero(n, n);
    Eigen::Matrix<double, n, m> B_d = Eigen::ArrayXXd::Zero(n, m);

    discretize_state_space_matrices(A_c, B_c, dt, A_d, B_d);

    return A_d * x + B_d * u;
}

void run_mpc() {
    // Setting up UDP server socket...
}

int main()
{

    t_stance_remainder_left = t_stance_remainder_right = t_stance;
    // Find largest index in plot_data and use the next one as file name for log files
    std::string path = "/home/loukas/Documents/cpp/walking_controller/plot_data/";
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (path.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            //printf ("%s\n", ent->d_name);
            std::string temp_filename = split_string(ent->d_name, '/').back();
            //std::cout << temp_filename << std::endl;

            int index = atoi(split_string(temp_filename, '.')[0].c_str());
            if(index > largest_index) {
                largest_index = index;
            }
            //std::cout << "Largest index: " << largest_index << std::endl;
        }
        closedir (dir);
    }
    else {
        /* could not open directory */
        perror ("Could not open directory to get latest plot file.");
    }

    filename = std::to_string(largest_index + 1);
    std::cout << "filename: " << filename << std::endl;

    // Initiate damping ratio matrix, desired natural frequency, orientation gains as well as desired trajectory to avoid null pointer
    h << 0.6, 0, 0,
         0, 0.6, 0,
         0, 0, 0.6;
    
    omega_desired << 8.0 * M_PI, 16.0 * M_PI, 8.0 * M_PI;

    Kp_orientation = 9 / 1.0;
    Kd_orientation = 0.15 / 1.0;

    pos_desired_left_leg << 0, 0, -1.115, 0, 0; // Cartesian xyz + euler roll and pitch
    vel_desired_left_leg << 0, 0, 0, 0, 0; // Cartesian xyz + euler roll and pitch
    accel_desired_left_leg << 0, 0, 0; // Cartesian xyz

    left_foot_pos_world << -0.15, 0, 0.6;
    right_foot_pos_world << 0.15, 0, 0.6;
    bool alternate_contacts = true;

    // auto start_test = high_resolution_clock::now();

    // auto traj = get_swing_trajectory((Eigen::Matrix<double,3,1>() << 0.1, 0.1, -1).finished(), (Eigen::Matrix<double,3,1>() << 0.05, 0.05, -0.9).finished(), 
    //                                     (Eigen::Matrix<double,3,1>() << 0, 0, -1).finished(), 
    //                                     (Eigen::Matrix<double,3,1>() << 0.0, 0.0, 0).finished(), (Eigen::Matrix<double,3,1>() << 0.0, 0.0, -0).finished(), 0.3);

    // auto end_test = high_resolution_clock::now();
    // std::cout << duration_cast<microseconds>(end_test - start_test).count() << std::endl;

    // ofstream traj_log_file;
    // traj_log_file.open(".././plot_data/traj_log_test.csv");
    // for(int row = 0; row < 1000; ++row) {
    //     for(int col = 0; col < 3; ++col) {
    //         traj_log_file << traj(row, col*2+0) << "," << traj(row, col*2+1) << ",";
    //     }
    //     traj_log_file << "\n";
    // }
    // traj_log_file.close();
    
    // Bind functions to threads
    //left_leg_state_thread = std::thread(std::bind(update_left_leg_state));
    left_leg_torque_thread = std::thread(std::bind(calculate_left_leg_torques));
    // right_leg_torque_thread = std::thread(std::bind(calculate_right_leg_torques));
    //mpc_thread = std::thread(std::bind(run_mpc));

    // while(true) { }
    std::cout << "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "If you're running this in a docker container, make sure to use the --net=host option when running it." << std::endl;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n" << std::endl;

    std::cout << "omega_desired is currently:" << omega_desired(0) << ", " << omega_desired(1) << ", " << omega_desired(2) << std::endl; // Print out current natural frequency
    std::cout << std::endl;

    int sockfd;
    char buffer[udp_buffer_size];
    struct sockaddr_in servaddr, cliaddr;
    
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    
    // Filling server information
    servaddr.sin_family = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    servaddr.sin_port = htons(mpc_port);
    
    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    
    int msg_length;
    socklen_t len;

    /*
    opts = {}
    opts["print_time"] = 1
    opts["expand"] = False
    opts['ipopt'] = {"max_iter":40, "print_level":3, "acceptable_tol":1e-7, "acceptable_obj_change_tol":1e-5}
    */

    Dict opts;
    Dict ipopt_opts;

    ipopt_opts["max_iter"] = 40; // Max allowed solver iterations
    ipopt_opts["print_level"] = 0;
    ipopt_opts["acceptable_tol"] = 1e-7;
    ipopt_opts["acceptable_obj_change_tol"] = 1e-5;

    opts["print_time"] = 0;
    opts["ipopt"] = ipopt_opts;
    opts["expand"] = false;

    Function solver = nlpsol("solver", "ipopt", "../nlp.so", opts); // Initialize solver with precompiled C code binary

    static std::map<std::string, DM> solver_arguments, solution;
    
    static const int num_constraint_bounds = n * (N+1) + m * N + N * 8; // n * (N+1) for initial state and dynamics at each time step, m * N for reaction forces at each time step, N * 8 for friction constraints
    static const int num_decision_variables = n * (N+1) + m * N;  // n * (N+1) for initial state and N future states + m*N for N control actions 
    static const int num_decision_variable_bounds = num_decision_variables; // Same as bounds

    std::cout << "Constraint Bounds length: " << num_constraint_bounds << std::endl;
    std::cout << "Decision variable bounds length: " << num_decision_variable_bounds << std::endl;
    std::cout << "Decision variables length: " << num_decision_variables << std::endl;

    DM lbg(num_constraint_bounds, 1);
    DM ubg(num_constraint_bounds, 1);

    DM lbx(num_decision_variable_bounds, 1);
    DM ubx(num_decision_variable_bounds, 1);

    // Initial state, Dynamics constraints and Contact constraints
    for(int i = 0; i < n * (N+1) + m*N; ++i) {
        lbg(i) = 0;
        ubg(i) = 0;
    }

    // Friction constraints
    for(int i = n * (N+1) + m*N; i < num_constraint_bounds; ++i) {
        lbg(i) = -DM::inf();
        ubg(i) = 0;
    }

    // State constraints (unbounded to avoid infeasibility)
    for(int i = 0; i < n*(N+1); ++i) {
        lbx(i) = -DM::inf();
        ubx(i) = DM::inf();
    }

    // Force constraints
    for(int i = 0; i < N; ++i) {
        int index = n*(N+1) + (i*m);

        lbx(index) = -DM::inf();
        lbx(index+1) = -DM::inf();
        lbx(index+2) = f_min_z;
        lbx(index+3) = -DM::inf();
        lbx(index+4) = -DM::inf();
        lbx(index+5) = f_min_z;

        ubx(index) = DM::inf();
        ubx(index+1) = DM::inf();
        ubx(index+2) = f_max_z;
        ubx(index+3) = DM::inf();
        ubx(index+4) = DM::inf();
        ubx(index+5) = f_max_z;
    }

    static std::vector<Eigen::Matrix<double, m, 1>> control_history = {Eigen::ArrayXXd::Zero(m, 1), Eigen::ArrayXXd::Zero(m, 1)};

    Eigen::Matrix<double, n*(N+1)+m*N, 1> x0_solver = Eigen::ArrayXXd::Zero(n*(N+1)+m*N, 1); // Full initial solver state, containing initial model state, N future states and N control actions
    Eigen::Matrix<double, n*(N+1), 1> X_t = Eigen::ArrayXXd::Zero(n*(N+1), 1); // Maybe this is actually obsolete and only x0_solver is sufficient
    
    static Eigen::Matrix<double, m*N, 1> U_t = Eigen::ArrayXXd::Zero(m*N, 1); // Same here

    static Eigen::Matrix<double, n, N> x_ref = Eigen::ArrayXXd::Zero(n, N); // N states "stacked" horizontally, containing the reference state trajectory for the prediction horizon

    static const int P_rows = n;
    static const int P_cols = 1 + N + n * N + m * N + N * m; // 1 for initial state, N for N reference states, N A matrices, N B matrices, N D matrices for contact
    static Eigen::Matrix<double, P_rows, P_cols> P_param = Eigen::ArrayXXd::Zero(P_rows, P_cols);

    Eigen::Matrix<double, 3, 3> I_world = Eigen::ArrayXXd::Zero(3, 3); // Body inertia in World frame

    static long long total_iterations = 0; // Total loop iterations

    // Desired state values
    double pos_x_desired = 0;
    double pos_y_desired = 0.0;
    double pos_z_desired = 1.48;

    double vel_x_desired = 0.0;
    double vel_y_desired = 0.0;
    double vel_z_desired = 0.0;

    double phi_desired = 0;
    double theta_desired = 0;
    double psi_desired = 0;

    double omega_x_desired = 0;
    double omega_y_desired = 0;
    double omega_z_desired = 0;

    double gait_gain = 0.1; // Rename to more accurate name

    double r_x_limit = 0.5; // The relative foot position in x (so r_x_left and r_x_right) is limited to +/- r_x_limit

    double hip_offset = 0.15; // Offset in x direction from Center of body frame / CoM to center of hip joint
    
    // Foot positions used in state space matrices. These are in the world frame, but represent a vector from the CoM! This means the formula to get the actual foot world position is r + p
    double r_x_left = -hip_offset;
    double r_y_left = 0;
    double r_z_left = -x_t(5, 0);

    double r_x_right = hip_offset;
    double r_y_right = 0;
    double r_z_right = -x_t(5, 0);

    Eigen::Matrix<double, n, n> A_c = Eigen::ArrayXXd::Zero(n, n); // A Matrix in Continuous time
    Eigen::Matrix<double, n, m> B_c = Eigen::ArrayXXd::Zero(n, m); // B Matrix in Continuous time
    
    Eigen::Matrix<double, 3, 3> r_left_skew_symmetric = Eigen::ArrayXXd::Zero(3, 3); // Skew symmetric version of the cross product with r. Needed because cross product cannot be a matrix element
    Eigen::Matrix<double, 3, 3> r_right_skew_symmetric = Eigen::ArrayXXd::Zero(3, 3); // Skew symmetric version of the cross product with r. Needed because cross product cannot be a matrix element

    Eigen::Matrix<double, m, m*N> D_vector = Eigen::ArrayXXd::Zero(m, m*N); // Vector containting N D matrices for contact constraints
    Eigen::Matrix<double, m, m> D_k = Eigen::ArrayXXd::Zero(m, m); // D Matrix at timestep k in Prediction Horizon

    solver_arguments["lbg"] = lbg; // Lower bounds on constraints
    solver_arguments["ubg"] = ubg; // Upper bounds on constraints
    solver_arguments["lbx"] = lbx; // Lower bounds on state
    solver_arguments["ubx"] = ubx; // Upper bounds on state

    // Log file
    ofstream data_file;
    data_file.open(".././plot_data/mpc_log.csv");
    data_file << "t,phi,theta,psi,pos_x,pos_y,pos_z,omega_x,omega_y,omega_z,vel_x,vel_y,vel_z,g,f_x_left,f_y_left,f_z_left,f_x_right,f_y_right,f_z_right,r_x_left,r_y_left,r_z_left,r_x_right,r_y_right,r_z_right,theta_delay_compensation,full_iteration_time,phi_delay_compensation" << std::endl; // Add header to csv file
    data_file.close();

    struct timespec deadline; // timespec struct for storing time that execution thread should sleep for

    Eigen::Matrix<double, 3, 1> left_foot_pos_world_discretization, right_foot_pos_world_discretization;

    while(true) {
        // Loop starts here
        auto start = high_resolution_clock::now();
        auto start_total = high_resolution_clock::now();

        // if (vel_y_desired < 0.2) {
        //     vel_y_desired += 0.01;
        // }

        // if(omega_z_desired < 0.8) {
        //     omega_z_desired += 0.02;
        // }

        // std::cout << "-----------------------------------------------------------------------------\nr_left at beginning of iteration: " << r_x_left << "," << r_y_left << "," << r_z_left << ", r_right at beginning of iteration: " << r_x_right << "," << r_y_right << "," << r_z_right << std::endl;

        msg_length = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, ( struct sockaddr *) &cliaddr, &len); // Receive message over UDP containing full leg state
        buffer[msg_length] = '\0'; // Add string ending delimiter to end of string (n is length of message)
        
        std::string raw_state(buffer); // Create string from buffer char array to split
        
        std::vector<std::string> com_state = split_string(raw_state, '|'); // Split raw state message by message delimiter to parse individual elements

        x_mutex.lock();
        for(int i = 0; i < n; ++i) {
            x_t(i, 0) = atof(com_state[i].c_str());
            //P_param(i,0) = x_t(i, 0);
        }
        x_mutex.unlock();

        // Step the model one timestep and use the resulting state as the initial state for the solver. This compensates for the roughly 1 sample delay due to the solver time
        P_param.block<n,1>(0, 0) = step_discrete_model(x_t, u_t, r_x_left, r_x_right, r_y_left, r_y_right, r_z_left, r_z_right);

        if (total_iterations == 0) {
            // Give solver better guess for first iteration to reduce solver time and generate more fitting solution
            for(int i = 0; i < (N+1); ++i) {
                for(int k = 0; k < n; ++k) {
                    X_t(k + n*i, 0) = P_param(k, 0);
                }
            }
        }

        std::cout << "x_t:" << x_t(0, 0) << "," << x_t(1, 0) << "," << x_t(2, 0) << "," << x_t(3, 0) << "," << x_t(4, 0) << "," << x_t(5, 0) << "," << x_t(6, 0) << "," << x_t(7, 0) << "," << x_t(8, 0) << "," << x_t(9, 0) << "," << x_t(10, 0) << "," << x_t(11, 0) << "," << x_t(12, 0) << std::endl;
        // Alternate contacts if contact_swap_interval iterations have passed
        if (total_iterations % contact_swap_interval == 0 && alternate_contacts) {
            t_stance_remainder_left_mutex.lock();
            t_stance_remainder_right_mutex.lock();

            // TODO: If I'm not missing anything, it should still work if reduced to only one variable, i.e. only lift_off_pos and lift_off_vel
            lift_off_pos_left_mutex.lock();
            lift_off_pos_right_mutex.lock();
            lift_off_vel_left_mutex.lock();
            lift_off_vel_right_mutex.lock();

            if(!swing_left) { // Left foot will now be in swing phase so we need to save lift off position for swing trajectory planning
                t_stance_remainder_left = t_stance;
                lift_off_pos_left = foot_pos_left_leg.block<3, 1>(0, 0);
                lift_off_vel_left = x_t.block<3, 1>(9, 0);
            }
            if(!swing_right) { // Right foot will now be in swing phase so we need to save lift off position for swing trajectory planning
                t_stance_remainder_right = t_stance;
                lift_off_pos_right = foot_pos_right_leg.block<3,1>(0, 0);
                lift_off_vel_right = x_t.block<3, 1>(9, 0);
            }

            lift_off_pos_left_mutex.unlock();
            lift_off_pos_right_mutex.unlock();
            lift_off_vel_left_mutex.unlock();
            lift_off_vel_right_mutex.unlock();
            
            swing_left = !swing_left;
            swing_right = !swing_right;
            // t_stance_remainder_left = t_stance_remainder_right = t_stance;
            // std::cout << t_stance_remainder_left << std::endl;
            t_stance_remainder_left_mutex.unlock();
            t_stance_remainder_right_mutex.unlock();
        }

        // Temporary variable for "simulating" future contacts
        bool swing_left_temp = swing_left;
        bool swing_right_temp = swing_right;
        // Loop through prediction horizon
        for(int k = 0; k < N; ++k) {
            // If contact_swap_interval iterations in future have passed, alternate again. the k != 0 check is there to prevent swapping twice if it swapped before simulating the future contacts already.
            if((total_iterations + k) % contact_swap_interval == 0 && k != 0 && alternate_contacts) {
                swing_left_temp = !swing_left_temp;
                swing_right_temp = !swing_right_temp;
            }

            D_k << swing_left_temp, 0, 0, 0, 0, 0,
                    0, swing_left_temp, 0, 0, 0, 0,
                    0, 0, swing_left_temp, 0, 0, 0,
                    0, 0, 0, swing_right_temp, 0, 0,
                    0, 0, 0, 0, swing_right_temp, 0,
                    0, 0, 0, 0, 0, swing_right_temp;
            
            D_vector.block<m, m>(0, k*m) = D_k;
        }
        
        // Update P_param
        P_param.block<m, m*N> (0, 1 + N + n*N + m*N) = D_vector;

        //Set U_ref depending on contact combination present
        for(int k = 0; k < N; ++k) {
            if(P_param(0, 1+N+n*N+m*N+k*m) == 1 && P_param(3, 1+N+n*N+m*N+k*m+3) == 1) { // No feet in contact
                P_param(m+0, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+1, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+2, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+3, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+4, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+5, 1 + N + n*N + m*N + k*m) = 0;
            }
            else if(P_param(0, 1+N+n*N+m*N+k*m) == 1 && P_param(3, 1+N+n*N+m*N+k*m+3) == 0) { // Right foot in contact
                P_param(m+0, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+1, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+2, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+3, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+4, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+5, 1 + N + n*N + m*N + k*m) = m_value * 9.81;
            }
            else if(P_param(0, 1+N+n*N+m*N+k*m) == 0 && P_param(3, 1+N+n*N+m*N+k*m+3) == 1) { // Left foot in contact
                P_param(m+0, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+1, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+2, 1 + N + n*N + m*N + k*m) = m_value * 9.81;
                P_param(m+3, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+4, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+5, 1 + N + n*N + m*N + k*m) = 0;
            }
            else if(P_param(0, 1+N+n*N+m*N+k*m) == 0 && P_param(3, 1+N+n*N+m*N+k*m+3) == 0) { // Both feet in contact
                P_param(m+0, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+1, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+2, 1 + N + n*N + m*N + k*m) = (m_value * 9.81) / 2;
                P_param(m+3, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+4, 1 + N + n*N + m*N + k*m) = 0;
                P_param(m+5, 1 + N + n*N + m*N + k*m) = (m_value * 9.81) / 2;
            }
        }
        
        if (total_iterations % contact_swap_interval == 0) {
            
            Eigen::Matrix<double, 4, 4> H_body_world; // Transformation matrix from body frame to world frame

            //ZYX order
            H_body_world << cos(P_param(2, 0))*cos(P_param(1, 0)), sin(P_param(0, 0))*sin(P_param(1, 0))*cos(P_param(2, 0)) - sin(P_param(2, 0))*cos(P_param(0, 0)), sin(P_param(0, 0))*sin(P_param(2, 0)) + sin(P_param(1, 0))*cos(P_param(0, 0))*cos(P_param(2, 0)), P_param(3, 0),
                                sin(P_param(2, 0))*cos(P_param(1, 0)), sin(P_param(0, 0))*sin(P_param(2, 0))*sin(P_param(1, 0)) + cos(P_param(0, 0))*cos(P_param(2, 0)), -sin(P_param(0, 0))*cos(P_param(2, 0)) + sin(P_param(2, 0))*sin(P_param(1, 0))*cos(P_param(0, 0)), P_param(4, 0),
                                -sin(P_param(1, 0)), sin(P_param(0, 0))*cos(P_param(1, 0)), cos(P_param(0, 0))*cos(P_param(1, 0)), P_param(5, 0),
                                0, 0, 0, 1;

            // Transform point in body frame to world frame to get current world position. hip_offset is the hip joint position
            Eigen::Matrix<double, 3, 1> adjusted_pos_vector_left = (H_body_world * (Eigen::Matrix<double, 4, 1>() << -hip_offset , 0, 0, 1).finished()).block<3,1>(0, 0);
            Eigen::Matrix<double, 3, 1> adjusted_pos_vector_right = (H_body_world * (Eigen::Matrix<double, 4, 1>() << hip_offset, 0, 0, 1).finished()).block<3, 1>(0, 0);
            
            Eigen::Matrix<double, 3, 1> vel_vector = (Eigen::Matrix<double, 3, 1>() << P_param(9, 0), P_param(10, 0), P_param(11, 0)).finished();
            Eigen::Matrix<double, 3, 1> vel_desired_vector = (Eigen::Matrix<double, 3, 1>() << vel_x_desired, vel_y_desired, vel_z_desired).finished();
            Eigen::Matrix<double, 3, 1> omega_desired_vector = (Eigen::Matrix<double, 3, 1>() << omega_x_desired, omega_y_desired, omega_z_desired).finished();
            
            left_foot_pos_world_mutex.lock();
            right_foot_pos_world_mutex.lock();
            left_foot_pos_world = adjusted_pos_vector_left + (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(P_param(5, 0)) / 9.81) * vel_vector.cross(omega_desired_vector);
            right_foot_pos_world = adjusted_pos_vector_right + (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(P_param(5, 0)) / 9.81) * vel_vector.cross(omega_desired_vector);

            left_foot_pos_world(2, 0) = right_foot_pos_world(2, 0) = 0; // This is needed because the formula above doesn't make sense for Z, and the foot naturally touches the ground at Z = 0 in the world frame

            //TODO: Instead of using inverse, either solve the inverse symbolically in python or just ues Transpose as shown in Modern Robotics Video
            // Find foot position in body frame to limit it in order to account for leg reachability, collision with other leg and reasonable values
            //H_body_world.inverse() is H_world_body
            Eigen::Matrix<double, 3, 1> left_foot_pos_body = (H_body_world.inverse() * (Eigen::Matrix<double, 4, 1>() << left_foot_pos_world, 1).finished()).block<3,1>(0, 0);
            Eigen::Matrix<double, 3, 1> right_foot_pos_body = (H_body_world.inverse() * (Eigen::Matrix<double, 4, 1>() << right_foot_pos_world, 1).finished()).block<3,1>(0,0);
            
            // Constrain X value of foot position in body coordinates
            if (left_foot_pos_body(0, 0) > r_x_limit - hip_offset) {
                left_foot_pos_body(0, 0) = r_x_limit - hip_offset;
                left_foot_pos_world = (H_body_world * (Eigen::Matrix<double, 4, 1>() << left_foot_pos_body, 1).finished()).block<3,1>(0, 0);
            }
            else if (left_foot_pos_body(0, 0) < -r_x_limit - hip_offset) {
                left_foot_pos_body(0, 0) = -r_x_limit - hip_offset;
                left_foot_pos_world = (H_body_world * (Eigen::Matrix<double, 4, 1>() << left_foot_pos_body, 1).finished()).block<3,1>(0, 0);
            }

            if (right_foot_pos_body(0, 0) > r_x_limit + hip_offset) {
                right_foot_pos_body(0, 0) = r_x_limit + hip_offset;
                right_foot_pos_world = (H_body_world * (Eigen::Matrix<double, 4, 1>() << right_foot_pos_body, 1).finished()).block<3,1>(0, 0);
            }
            else if (right_foot_pos_body(0, 0) < -r_x_limit + hip_offset) {
                right_foot_pos_body(0, 0) = -r_x_limit + hip_offset;
                right_foot_pos_world = (H_body_world * (Eigen::Matrix<double, 4, 1>() << right_foot_pos_body, 1).finished()).block<3,1>(0, 0);
            }

            std::cout << "left_foot_pos_world: " << left_foot_pos_world(0, 0) << "," << left_foot_pos_world(1, 0) << "," << left_foot_pos_world(2, 0) << std::endl;
        }

        // Calculate r from foot world position
        r_x_left = left_foot_pos_world(0, 0) - (double)P_param(3, 0);
        r_x_right = right_foot_pos_world(0, 0) - (double)P_param(3, 0);

        r_y_left = left_foot_pos_world(1, 0) - (double)P_param(4, 0);
        r_y_right = right_foot_pos_world(1, 0) - (double)P_param(4, 0);

        left_foot_pos_world_mutex.unlock();
        right_foot_pos_world_mutex.unlock();

        r_z_left = -P_param(5, 0);
        r_z_right = -P_param(5, 0);

        double pos_x_desired_temp = pos_x_desired;
        double pos_y_desired_temp = pos_y_desired;
        double vel_y_desired_temp = vel_y_desired;// - 0.01;
        double pos_z_desired_temp = pos_z_desired;

        double phi_desired_temp = phi_desired;
        double theta_desired_temp = theta_desired;
        double psi_desired_temp = psi_desired;
        double omega_z_desired_temp = omega_z_desired;// - 0.02;

        //Update reference trajectory

        for(int i = 0; i < N; ++i) {
            // if (vel_y_desired_temp < 0.2) {
            //     vel_y_desired_temp += 0.02;
            // }

            // if (omega_z_desired_temp < 0.8) {
            //     omega_z_desired_temp += 0.02;
            // }

            pos_x_desired_temp += vel_x_desired * dt;
            pos_y_desired_temp += vel_y_desired_temp * dt;
            pos_z_desired_temp += vel_z_desired * dt;

            phi_desired_temp += omega_x_desired * dt;
            theta_desired_temp += omega_y_desired * dt;
            psi_desired_temp += omega_z_desired_temp * dt;

            x_ref(0, i) = phi_desired_temp; // Roll
            x_ref(1, i) = theta_desired_temp; // Pitch
            x_ref(2, i) = psi_desired_temp; // Yaw
            x_ref(3, i) = pos_x_desired_temp; // X Pos
            x_ref(4, i) = pos_y_desired_temp; // Y Pos
            x_ref(5, i) = pos_z_desired_temp; // Z Pos
            x_ref(6, i) = omega_x_desired; // Omega_x
            x_ref(7, i) = omega_y_desired; // Omega_y
            x_ref(8, i) = omega_z_desired_temp; // Omega_z
            x_ref(9, i) = vel_x_desired; // X Vel
            x_ref(10, i) = vel_y_desired_temp; // Y Vel
            x_ref(11, i) = vel_z_desired; // Z Vel
            x_ref(12, i) = -9.81; // Gravity constant
        }

        P_param.block<n, N>(0, 1) = x_ref;

        pos_x_desired += vel_x_desired * dt;
        pos_y_desired += vel_y_desired * dt;
        pos_z_desired += vel_z_desired * dt;

        phi_desired += omega_x_desired * dt;
        theta_desired += omega_y_desired * dt;
        psi_desired += omega_z_desired * dt;

        double r_x_left_prev = r_x_left;
        double r_x_right_prev = r_x_right;

        double r_y_left_prev = r_y_left;
        double r_y_right_prev = r_y_right;

        left_foot_pos_world_mutex.lock();
        right_foot_pos_world_mutex.lock();
        left_foot_pos_world_discretization = left_foot_pos_world;
        right_foot_pos_world_discretization = right_foot_pos_world;
        left_foot_pos_world_mutex.unlock();
        right_foot_pos_world_mutex.unlock();

        double phi_t = 0.0;
        double theta_t = 0.0;
        double psi_t = 0.0;

        double vel_x_t = 0.0;
        double vel_y_t = 0.0;
        double vel_z_t = 0.0;
        
        double pos_x_t = 0.0;
        double pos_y_t = 0.0;
        double pos_y_t_next = 0.0;
        double pos_z_t = 0.0;

        // Discretization loop for Prediction Horizon
        for(int i = 0; i < N; ++i) {
            if (i < N-1) {
                phi_t = X_t(n*(i+1) + 0, 0);
                theta_t = X_t(n*(i+1) + 1, 0);
                psi_t = X_t(n*(i+1) + 2 ,0);

                vel_x_t = X_t(n*(i+1)+9, 0);
                vel_y_t = X_t(n*(i+1)+10, 0);
                vel_z_t = X_t(n*(i+1)+11, 0);
                
                pos_x_t = X_t(n*(i+1)+3, 0);
                pos_y_t = X_t(n*(i+1)+4, 0);
                pos_y_t_next = X_t(n*(i+2)+4, 0);
                pos_z_t = X_t(n*(i+1)+5, 0);
            }
            else {
                phi_t = X_t(n*(N-1) + 0, 0);
                theta_t = X_t(n*(N-1) + 1, 0);
                psi_t = X_t(n*(N-1) + 2, 0);

                vel_x_t = X_t(n*(N-1)+9, 0);
                vel_y_t = X_t(n*(N-1)+10, 0);
                vel_z_t = X_t(n*(N-1)+11, 0);
                
                pos_x_t = X_t(n*(N-1) + 3, 0);
                pos_y_t = X_t(n*(N-1) + 4, 0);
                pos_y_t_next = X_t(n*N + 4, 0);
                pos_z_t = X_t(n*(N-1) + 5, 0);
            }

            if(i == 0) {
                phi_t = (double)P_param(0, 0);
                theta_t = (double)P_param(1, 0);
                psi_t = (double)P_param(2, 0);

                vel_x_t = (double)P_param(9, 0);
                vel_y_t = (double)P_param(10, 0);
                vel_z_t = (double)P_param(11, 0);

                pos_x_t = (double)P_param(3, 0);
                pos_y_t = (double)P_param(4, 0);
                pos_z_t = (double)P_param(5, 0);
            }

            int swap_counter = 0;

            if((total_iterations+i) % contact_swap_interval == 0 && i != 0) {
                
                // See comments before, same procedure here, also ZYX order
                Eigen::Matrix<double, 4, 4> H_body_world = (Eigen::Matrix<double, 4, 4>() << cos(psi_t)*cos(theta_t), sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t), sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t), pos_x_t,
                                                            sin(psi_t)*cos(theta_t), sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t), -sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t), pos_y_t,
                                                            -sin(theta_t), sin(phi_t)*cos(theta_t), cos(phi_t)*cos(theta_t), pos_z_t,
                                                            0, 0, 0, 1).finished();

                Eigen::Matrix<double, 3, 1> adjusted_pos_vector_left = (H_body_world * (Eigen::Matrix<double, 4, 1>() << -hip_offset, 0, 0, 1).finished()).block<3,1>(0, 0);
                Eigen::Matrix<double, 3, 1> adjusted_pos_vector_right = (H_body_world * (Eigen::Matrix<double, 4, 1>() << hip_offset, 0, 0, 1).finished()).block<3, 1>(0, 0);
                
                Eigen::Matrix<double, 3, 1> vel_vector = (Eigen::Matrix<double, 3, 1>() << vel_x_t, vel_y_t, vel_z_t).finished();
                Eigen::Matrix<double, 3, 1> vel_desired_vector = (Eigen::Matrix<double, 3, 1>() << vel_x_desired, vel_y_desired, vel_z_desired).finished();
                Eigen::Matrix<double, 3, 1> omega_desired_vector = (Eigen::Matrix<double, 3, 1>() << omega_x_desired, omega_y_desired, omega_z_desired).finished();

                left_foot_pos_world_discretization = adjusted_pos_vector_left + (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(pos_z_t) / 9.81) * vel_vector.cross(omega_desired_vector);
                right_foot_pos_world_discretization = adjusted_pos_vector_right + (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(pos_z_t) / 9.81) * vel_vector.cross(omega_desired_vector);

                left_foot_pos_world_discretization(2, 0) = right_foot_pos_world_discretization(2, 0) = 0; // This is needed because the formula above doesn't make sense for Z, and the foot naturally touches the ground at Z = 0 in the world frame
                
                //TODO: Instead of using inverse, either solve the inverse symbolically in python or just ues Transpose as shown in Modern Robotics Video
                Eigen::Matrix<double, 3, 1> left_foot_pos_body = (H_body_world.inverse() * (Eigen::Matrix<double, 4, 1>() << left_foot_pos_world_discretization, 1).finished()).block<3,1>(0, 0);
                Eigen::Matrix<double, 3, 1> right_foot_pos_body = (H_body_world.inverse() * (Eigen::Matrix<double, 4, 1>() << right_foot_pos_world_discretization, 1).finished()).block<3,1>(0, 0);

               // Constrain X value of foot position in body coordinates, TODO: Do this in all directions with a smarter implementation using precise reachability of leg kinematics (account for singularities as well)
                if (left_foot_pos_body(0, 0) > r_x_limit - hip_offset) {
                    left_foot_pos_body(0, 0) = r_x_limit - hip_offset;
                    left_foot_pos_world_discretization = (H_body_world * (Eigen::Matrix<double, 4, 1>() << left_foot_pos_body, 1).finished()).block<3,1>(0, 0);
                }
                else if (left_foot_pos_body(0, 0) < -r_x_limit - hip_offset) {
                    left_foot_pos_body(0, 0) = -r_x_limit - hip_offset;
                    left_foot_pos_world_discretization = (H_body_world * (Eigen::Matrix<double, 4, 1>() << left_foot_pos_body, 1).finished()).block<3,1>(0, 0);
                }

                if (right_foot_pos_body(0, 0) > r_x_limit + hip_offset) {
                    right_foot_pos_body(0, 0) = r_x_limit + hip_offset;
                    right_foot_pos_world_discretization = (H_body_world * (Eigen::Matrix<double, 4, 1>() << right_foot_pos_body, 1).finished()).block<3,1>(0, 0);
                }
                else if (right_foot_pos_body(0, 0) < -r_x_limit + hip_offset) {
                    right_foot_pos_body(0, 0) = -r_x_limit + hip_offset;
                    right_foot_pos_world_discretization = (H_body_world * (Eigen::Matrix<double, 4, 1>() << right_foot_pos_body, 1).finished()).block<3,1>(0, 0);
                }
                
                if(swap_counter < 1) {
                    left_foot_pos_desired_world_mutex.lock();
                    right_foot_pos_desired_world_mutex.lock();
                    next_body_vel_mutex.lock();

                    left_foot_pos_desired_world = left_foot_pos_world_discretization;
                    right_foot_pos_desired_world = right_foot_pos_world_discretization;
                    next_body_vel = (Eigen::Matrix<double, 3, 1>() << vel_x_t, vel_y_t, vel_z_t).finished();
                    
                    left_foot_pos_desired_world_mutex.unlock();
                    right_foot_pos_desired_world_mutex.unlock();
                    next_body_vel_mutex.unlock();
                }

                swap_counter++;
            }

            r_x_left = left_foot_pos_world_discretization(0, 0) - pos_x_t;
            r_x_right = right_foot_pos_world_discretization(0, 0) - pos_x_t;

            r_y_left = left_foot_pos_world_discretization(1, 0) - pos_y_t;
            r_y_right = right_foot_pos_world_discretization(1, 0) - pos_y_t;

            r_z_left = -pos_z_t;
            r_z_right = -pos_z_t;

            // Calculate I_world based on formula from notebook, which equates to R_zyx * I_world * R_zyx.T
            I_world << (sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixz*cos(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izz*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))) + (sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))*(Ixy*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))) + (Ixx*cos(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izx*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)))*cos(psi_t)*cos(theta_t), (-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixz*cos(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izz*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))) + (sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))) + (Ixx*cos(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izx*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)))*sin(psi_t)*cos(theta_t), (Ixy*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)))*sin(phi_t)*cos(theta_t) - (Ixx*cos(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izx*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)))*sin(theta_t) + (Ixz*cos(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t)) + Izz*(sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t)))*cos(phi_t)*cos(theta_t),
                        (sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixz*sin(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izz*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))) + (sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))*(Ixy*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Ixy*sin(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))) + (Ixx*sin(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izx*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*cos(psi_t)*cos(theta_t), (-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixz*sin(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izz*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))) + (sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Ixy*sin(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))) + (Ixx*sin(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izx*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*sin(psi_t)*cos(theta_t), (Ixy*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Ixy*sin(psi_t)*cos(theta_t) + Iyy*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)))*sin(phi_t)*cos(theta_t) - (Ixx*sin(psi_t)*cos(theta_t) + Iyx*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izx*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*sin(theta_t) + (Ixz*sin(psi_t)*cos(theta_t) + Iyz*(sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) + Izz*(-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*cos(phi_t)*cos(theta_t),
                        (sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t))*(-Ixz*sin(theta_t) + Iyz*sin(phi_t)*cos(theta_t) + Izz*cos(phi_t)*cos(theta_t)) + (sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t))*(-Ixy*sin(theta_t) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*sin(phi_t)*cos(theta_t)) + (-Ixx*sin(theta_t) + Iyx*sin(phi_t)*cos(theta_t) + Izx*cos(phi_t)*cos(theta_t))*cos(psi_t)*cos(theta_t), (-sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(-Ixz*sin(theta_t) + Iyz*sin(phi_t)*cos(theta_t) + Izz*cos(phi_t)*cos(theta_t)) + (sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(-Ixy*sin(theta_t) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*sin(phi_t)*cos(theta_t)) + (-Ixx*sin(theta_t) + Iyx*sin(phi_t)*cos(theta_t) + Izx*cos(phi_t)*cos(theta_t))*sin(psi_t)*cos(theta_t), -(-Ixx*sin(theta_t) + Iyx*sin(phi_t)*cos(theta_t) + Izx*cos(phi_t)*cos(theta_t))*sin(theta_t) + (-Ixy*sin(theta_t) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*sin(phi_t)*cos(theta_t))*sin(phi_t)*cos(theta_t) + (-Ixz*sin(theta_t) + Iyz*sin(phi_t)*cos(theta_t) + Izz*cos(phi_t)*cos(theta_t))*cos(phi_t)*cos(theta_t);
            
            //TODO: Maybe use functions for this
            r_left_skew_symmetric << 0, -r_z_left, r_y_left,
                                        r_z_left, 0, -r_x_left,
                                        -r_y_left, r_x_left, 0;
            
            r_right_skew_symmetric << 0, -r_z_right, r_y_right,
                                        r_z_right, 0, -r_x_right,
                                        -r_y_right, r_x_right, 0;
            
            // See MIT Paper for Rotation Matrix explanation
            A_c << 0, 0, 0, 0, 0, 0, cos(psi_t) / cos(theta_t), sin(psi_t) / cos(theta_t), 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, -sin(psi_t), cos(psi_t), 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, cos(psi_t) * tan(theta_t), sin(psi_t)*tan(theta_t), 1, 0, 0, 0, 0,
                    
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                    
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                    
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

            B_c << 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    I_world.inverse() * r_left_skew_symmetric, I_world.inverse() * r_right_skew_symmetric,
                    1/m_value, 0, 0, 1/m_value, 0, 0,
                    0, 1/m_value, 0, 0, 1/m_value, 0,
                    0, 0, 1/m_value, 0, 0, 1/m_value,
                    0, 0, 0, 0, 0, 0;

            Eigen::Matrix<double, n, n> A_d_t = Eigen::ArrayXXd::Zero(n, n);
            Eigen::Matrix<double, n, m> B_d_t = Eigen::ArrayXXd::Zero(n, m);

            discretize_state_space_matrices(A_c, B_c, dt, A_d_t, B_d_t); // Actually discretize continuous state space matrices
            // Copy them over to P_Param used for solver
            P_param.block<n, n>(0, 1 + N + (i*n)) = A_d_t;
            P_param.block<n, m>(0, 1 + N + n * N + (i*m)) = B_d_t;
        }

        r_x_left = r_x_left_prev;
        r_x_right = r_x_right_prev;

        r_y_left = r_y_left_prev;
        r_y_right = r_y_right_prev;

        r_z_left = r_z_right = -P_param(5, 0);

        update_left_leg_foot_trajectory();

        // Copy all values from Eigen Matrices to casadi DM because that's what casadi accepts for the solver
        size_t rows_P_param = P_param.rows();
        size_t cols_P_param = P_param.cols();

        DM P_param_casadi = casadi::DM::zeros(rows_P_param, cols_P_param);

        std::memcpy(P_param_casadi.ptr(), P_param.data(), sizeof(double)*rows_P_param*cols_P_param);
        
        solver_arguments["p"] = P_param_casadi;
        x0_solver << X_t, U_t;

        size_t rows_x0_solver = x0_solver.rows();
        size_t cols_x0_solver = x0_solver.cols();

        DM x0_solver_casadi = casadi::DM::zeros(rows_x0_solver, cols_x0_solver);

        std::memcpy(x0_solver_casadi.ptr(), x0_solver.data(), sizeof(double)*rows_x0_solver*cols_x0_solver);
        solver_arguments["x0"] = x0_solver_casadi;

        auto end = high_resolution_clock::now();

        double duration_before = duration_cast<microseconds>(end - start).count();

        auto sol_start = high_resolution_clock::now();

        solution = solver(solver_arguments); // Solve the NLP

        auto sol_end = high_resolution_clock::now();
        double solver_time = duration_cast<microseconds>(sol_end - sol_start).count();

        start = high_resolution_clock::now();
        // Extract u_t (optimal first control action) from solution
        size_t rows = solution.at("x").size1();
        size_t cols = solution.at("x").size2();

        Eigen::Matrix<double, n*(N+1) + m*N, 1> solution_variables = Eigen::ArrayXXd::Zero(n*(N+1) + m*N, 1);

        solution_variables.resize(rows,cols);
        solution_variables.setZero(rows,cols);

        std::memcpy(solution_variables.data(), solution.at("x").ptr(), sizeof(double)*rows*cols);

        u_mutex.lock();
        x_mutex.lock();

        u_t << solution_variables(n*(N+1)+0),
                solution_variables(n*(N+1)+1),
                solution_variables(n*(N+1)+2),
                solution_variables(n*(N+1)+3),
                solution_variables(n*(N+1)+4),
                solution_variables(n*(N+1)+5);
        
        // Send optimal control over UDP, along with logging info for the gazebo plugin
        stringstream s;
        s << u_t(0, 0) << "|" << u_t(1, 0) << "|" << u_t(2, 0) << "|" << u_t(3, 0) << "|" << u_t(4, 0) << "|" << u_t(5, 0) << "|" << r_x_left << "|" << r_y_left << "|" << r_z_left << "|" << r_x_right << "|" << r_y_right << "|" << r_z_right << "|" << P_param(1, 0) << "|420|" << solution_variables(0, 0); // Write torque setpoints to stringstream
        // s << u_t(0, 0) << "|" << u_t(1, 0) << "|" << u_t(2, 0) << "|" << u_t(3, 0) << "|" << u_t(4, 0) << "|" << u_t(5, 0) << "|" << r_x_left << "|" << r_y_left << "|" << r_z_left << "|" << r_x_right << "|" << r_y_right << "|" << r_z_right << "|" << P_param(1, 0) << "|420|0" ; // Write torque setpoints to stringstream
        
        // std::cout << "u_t: " << u_t(0) << "," << u_t(1) << "," << u_t(2) << "," << u_t(3) << "," << u_t(4) << "," << u_t(5) << std::endl;

        u_mutex.unlock();
        x_mutex.unlock();
        
        sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len);
        
        next_body_vel_mutex.lock();
        next_body_vel = X_t.block<3, 1>(n*N+9, 0); // Update predicted velocity at next t_stance for impedance control thread calculating swing trajectory
        next_body_vel_mutex.unlock();

        // Use this solution for the next iteration as a hotstart, only shifted one timestep
        X_t.block<n*N, 1>(0, 0) = solution_variables.block<n*N, 1>(n, 0);
        X_t.block<n, 1>(n*N, 0) = solution_variables.block<n, 1>(n*N, 0);

        U_t.block<m*(N-1), 1>(0, 0) = solution_variables.block<m*(N-1), 1>(n*(N+1) + m, 0);
        U_t.block<m, 1>(m*(N-1), 0) = solution_variables.block<m, 1>(n*(N+1)+m*(N-1), 0);

        ++total_iterations;

        end = high_resolution_clock::now();
        double duration_after = duration_cast<microseconds> (end - start).count();

        // std::cout << "Solver preparation took " << duration_before + duration_after << " microseconds" << std::endl;

        auto end_total = high_resolution_clock::now();
        double full_iteration_duration = duration_cast<microseconds> (end_total - start_total).count();

        // std::cout << "Full iteration took " << full_iteration_du ration << " microseconds" << std::endl;
        
        u_mutex.lock();
        x_mutex.lock();

        // Log data to csv file
        ofstream data_file;
        data_file.open(".././plot_data/mpc_log.csv", ios::app); // Open csv file in append mode
        data_file << total_iterations * dt << "," << x_t(0, 0) << "," << x_t(1, 0) << "," << x_t(2, 0) << "," << x_t(3, 0) << "," << x_t(4, 0) << "," << x_t(5, 0) << "," << x_t(6, 0) << "," << x_t(7, 0) << "," << x_t(8, 0) << "," << x_t(9, 0) << "," << x_t(10, 0) << "," << x_t(11, 0) << "," << x_t(12, 0)
                << "," << u_t(0) << "," << u_t(1) << "," << u_t(2) << "," << u_t(3) << "," << u_t(4) << "," << u_t(5) 
                << "," << r_x_left << "," << r_y_left << "," << r_z_left << "," << r_x_right << "," << r_y_right << "," << r_z_right << "," << P_param(1, 0) << "," << full_iteration_duration / 1000.0 << "," << solution_variables(n, 0) << std::endl; // Zero at the end has to be replace with predicted delay compensation state!
        data_file.close(); // Close csv file again. This way thread abort should (almost) never leave file open.

        u_mutex.unlock();
        x_mutex.unlock();

        long long remainder = (dt * 1e+6 - full_iteration_duration) * 1e+3;
        //std::cout << "Remainder: " << remainder << " microseconds" << std::endl;
        deadline.tv_nsec = remainder;
        deadline.tv_sec = 0;
        clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
    }

    while(true) {

    }

    return 0;
}