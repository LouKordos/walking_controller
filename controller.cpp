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

// ZCM communication channel names

//zcm::ZCM zcm_context { "ipc" };

const char* left_leg_state_channel = ".left_leg_state";
const char* right_leg_state_channel = ".right_leg_state";

const char* left_leg_torque_setpoint_channel = "left_leg_torque_setpoint";
const char* right_leg_torque_setpoint_channel = "right_leg_torque_setpoint";

const int udp_port = 4200; // UDP port for communication between gazebosim and controller code
const int mpc_port = 4801;

const int udp_buffer_size = 4096; // Buffer size for receiving leg state from gazebosim

// Helper function for splitting string by delimiter character

std::vector<std::string> split_string(std::string str, char delimiter) {
    std::vector<std::string> results;

    boost::split(results, str, [&delimiter](char c){return c == delimiter;});

    return results;
}

// Helper function for constraining a double precision float to limits and filtering out "nan" and "inf"

inline void constrain(double &value, double lower_limit, double upper_limit) {
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

std::thread left_leg_state_thread; // Thread for updating left leg state based on gazebosim messages
std::thread left_leg_torque_thread; // Thread for updating matrices, calculating torque setpoint and sending torque setpoint to gazebosim
std::thread left_leg_mpc_thread;

double state_update_interval = 1000.0; // Interval for fetching and parsing the leg state from gazebosim in microseconds
double torque_calculation_interval = 1000.0; // Interval for calculating and sending the torque setpoint to gazebosim in microseconds

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
    servaddr.sin_port = htons(udp_port); 
      
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE);
    } 
      
    int n; 

    socklen_t len;

    // Setting up debugging and plotting csv file

    std::string path = "/home/loukas/Documents/cpp/walking_controller/plot_data/";
    int largest_index = 0;

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

    std::string filename = std::to_string(largest_index + 1);

    ofstream data_file;
    data_file.open(".././plot_data/" + filename + ".csv");
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

        double phi_t = 0; // Current desired roll orientation for the foot described as an Euler Angle
        double phi_dot_t = 0; // Current desired roll angular velocity for the foot described as an Euler Angle

        double psi_t = 0; // Current desired yaw orientation for the foot described as an Euler Angle
        double psi_dot_t = 0; // Current desired yaw angular velocity for the foot described as an Euler Angle

        // Updating desired trajectory

        x_pos_t = 0;
        x_vel_t = 0;
        x_accel_t = 0;
        
        double omega = 8.0; // Frequency for sinusoidal Trajectory in rad/s

        // X:

        //x_pos_t = 0.200000000000000011102L*cosl(2*t);
        x_pos_t = 0;
        x_vel_t = 0;
        x_accel_t = 0;

        // Y:

        y_pos_t = 0.200000000000000011102L*cosl(omega*t);
        y_vel_t = -0.200000000000000011102L*omega*sinl(omega*t);
        y_accel_t = -0.200000000000000011102L*powl(omega, 2)*cosl(omega*t);

        // Z:

        z_pos_t = 0.100000000000000005551L*sinl(omega*t) - 0.800000000000000044409L;
        z_vel_t = 0.100000000000000005551L*omega*cosl(omega*t);
        z_accel_t = -0.100000000000000005551L*powl(omega, 2)*sinl(omega*t);

        pos_desired_left_leg << x_pos_t, y_pos_t, z_pos_t, phi_t, psi_t;
        vel_desired_left_leg << x_vel_t, y_vel_t, z_vel_t, psi_t, psi_dot_t;
        accel_desired_left_leg << x_accel_t, y_accel_t, z_accel_t;

        // std::cout << "pos: " << pos_desired_left_leg(0) << ", " << pos_desired_left_leg(1) << ", " << pos_desired_left_leg(2) << ", vel: " << vel_desired_left_leg(0)
        //             << ", " << vel_desired_left_leg(1) << ", " << vel_desired_left_leg(2) 
        //             << ", accel: " << accel_desired_left_leg(0) << ", " << accel_desired_left_leg(1) << ", " << accel_desired_left_leg(2) << std::endl;

        //TODO: Maybe rework to only use q and q_dot

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

        n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, ( struct sockaddr *) &cliaddr, &len); // Receive message over UDP containing full leg state
        buffer[n] = '\0'; // Add string ending delimiter to end of string (n is length of message)

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

        // Update matrices based on received angles and angular velocities

        update_orientation(theta1, theta2, theta3, theta4, theta5);

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

        std::cout << "Foot Position: " << foot_pos_left_leg(0) << ", " << foot_pos_left_leg(1) << ", " << foot_pos_left_leg(2) << std::endl;

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
            data_file.open(".././plot_data/" + filename + ".csv", ios::app); // Open csv file in append mode
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

        //std::cout << "tau_ff: " << tau_ff_left_leg(0) << ", " << tau_ff_left_leg(1) << ", " << tau_ff_left_leg(2) << ", " << tau_ff_left_leg(3) << ", " << tau_ff_left_leg(4) << std::endl;
        //std::cout << "C*q_dot: " << C_left_leg(0) << ", " << C_left_leg(1) << ", " << C_left_leg(2) << ", " << C_left_leg(3) << ", " << C_left_leg(4) << std::endl;

        stringstream s;

        s << tau_setpoint_left_leg(0) << "|" << tau_setpoint_left_leg(1) << "|" << tau_setpoint_left_leg(2) << "|" << tau_setpoint_left_leg(3) << "|" << tau_setpoint_left_leg(4); // Write torque setpoints to stringstream

        sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), 
            MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len); // Send the torque setpoint string to the simulation

        iteration_counter++; // Increment iteration counter

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

static const int n = 13;
static const int m = 6;

static const double dt = 1/30.0;
static const int N = 30;

double f_min_z = 0;
double f_max_z = 1000;

static const double m_value = 30.0; // kg

static const Eigen::MatrixXd I_body = (Eigen::Matrix<double, 3, 3>() << 0.1536, 0.0, 0.0,
                                                                        0.0, 0.62288, 0.0,
                                                                        0.0, 0.0, 0.6843).finished();

static const double Ixx = I_body(0, 0);
static const double Ixy = I_body(0, 1);
static const double Ixz = I_body(0, 2);

static const double Iyx = I_body(1, 0);
static const double Iyy = I_body(1, 1);
static const double Iyz = I_body(1, 2);

static const double Izx = I_body(2, 0);
static const double Izy = I_body(2, 1);
static const double Izz = I_body(2, 2);

void discretize_state_space_matrices(Eigen::Matrix<double, n, n> &A_c_temp, Eigen::Matrix<double, n, m> &B_c_temp, const double &dt, Eigen::Matrix<double, n, n> &A_d_temp, Eigen::Matrix<double, n, m> &B_d_temp) {
    static Eigen::Matrix<double, n+m, n+m> A_B = Eigen::ArrayXXd::Zero(n+m, n+m);

    A_B.block<n, n>(0, 0) = A_c_temp;
    A_B.block<n, m>(0, n) = B_c_temp;
    //A_B << A_c_temp, B_c_temp, Eigen::ArrayXXd::Zero(m, n), Eigen::ArrayXXd::Zero(m, m);
    Eigen::MatrixXd e_A_B = (A_B * dt).exp();

    //std::cout << "A_B:\n" << A_B << std::endl;

    A_d_temp = e_A_B.block<n, n>(0, 0);
    B_d_temp = e_A_B.block<n, m>(0, n);

    //std::cout << "A_d_temp:\n" << A_d_temp << std::endl;
}

Eigen::Matrix<double, n, 1> step_discrete_model(Eigen::Matrix<double, n, 1> x, Eigen::Matrix<double, m, 1> u, double &r_x_left, double &r_x_right, double &r_y_left, double &r_y_right, double &r_z_left, double &r_z_right) {

    double phi_t = x(0);
    double theta_t = x(1);
    double psi_t = x(2);

    Eigen::Matrix<double, 3, 3> I_world;

    I_world << -(Ixy*sin(theta_t) + Ixy*cos(psi_t)*cos(theta_t) - Iyy*sin(psi_t)*cos(theta_t))*sin(psi_t)*cos(theta_t) + (Ixx*cos(psi_t)*cos(theta_t) - Iyx*sin(psi_t)*cos(theta_t) + Izx*sin(theta_t))*cos(psi_t)*cos(theta_t) + (Ixz*cos(psi_t)*cos(theta_t) - Iyz*sin(psi_t)*cos(theta_t) + Izz*sin(theta_t))*sin(theta_t), (-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*sin(theta_t) + Ixy*cos(psi_t)*cos(theta_t) - Iyy*sin(psi_t)*cos(theta_t)) + (sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t))*(Ixx*cos(psi_t)*cos(theta_t) - Iyx*sin(psi_t)*cos(theta_t) + Izx*sin(theta_t)) - (Ixz*cos(psi_t)*cos(theta_t) - Iyz*sin(psi_t)*cos(theta_t) + Izz*sin(theta_t))*sin(phi_t)*cos(theta_t), (sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixx*cos(psi_t)*cos(theta_t) - Iyx*sin(psi_t)*cos(theta_t) + Izx*sin(theta_t)) + (sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixy*sin(theta_t) + Ixy*cos(psi_t)*cos(theta_t) - Iyy*sin(psi_t)*cos(theta_t)) + (Ixz*cos(psi_t)*cos(theta_t) - Iyz*sin(psi_t)*cos(theta_t) + Izz*sin(theta_t))*cos(phi_t)*cos(theta_t),
                (Ixx*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyx*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izx*sin(phi_t)*cos(theta_t))*cos(psi_t)*cos(theta_t) - (Ixy*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) - Ixy*sin(phi_t)*cos(theta_t) + Iyy*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)))*sin(psi_t)*cos(theta_t) + (Ixz*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyz*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izz*sin(phi_t)*cos(theta_t))*sin(theta_t), (-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) - Ixy*sin(phi_t)*cos(theta_t) + Iyy*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))) + (sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t))*(Ixx*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyx*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izx*sin(phi_t)*cos(theta_t)) - (Ixz*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyz*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izz*sin(phi_t)*cos(theta_t))*sin(phi_t)*cos(theta_t), (sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixx*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyx*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izx*sin(phi_t)*cos(theta_t)) + (sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixy*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) - Ixy*sin(phi_t)*cos(theta_t) + Iyy*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))) + (Ixz*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyz*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izz*sin(phi_t)*cos(theta_t))*cos(phi_t)*cos(theta_t), 
                (Ixx*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyx*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izx*cos(phi_t)*cos(theta_t))*cos(psi_t)*cos(theta_t) - (Ixy*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*sin(psi_t)*cos(theta_t) + (Ixz*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyz*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izz*cos(phi_t)*cos(theta_t))*sin(theta_t), (-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))) + (sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t))*(Ixx*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyx*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izx*cos(phi_t)*cos(theta_t)) - (Ixz*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyz*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izz*cos(phi_t)*cos(theta_t))*sin(phi_t)*cos(theta_t), (sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixx*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyx*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izx*cos(phi_t)*cos(theta_t)) + (sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixy*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))) + (Ixz*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyz*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izz*cos(phi_t)*cos(theta_t))*cos(phi_t)*cos(theta_t);

    Eigen::Matrix<double, 3, 3> r_left_skew_symmetric_test, r_right_skew_symmetric_test;

    r_left_skew_symmetric_test << 0, -r_z_left, r_y_left,
                                r_z_left, 0, -r_x_left,
                                -r_y_left, r_x_left, 0;
            
    r_right_skew_symmetric_test << 0, -r_z_right, r_y_right,
                                r_z_right, 0, -r_x_right,
                                -r_y_right, r_x_right, 0;

    Eigen::Matrix<double, n, n> A_c;
    Eigen::Matrix<double, n, m> B_c;

    A_c << 0, 0, 0, 0, 0, 0, cos(psi_t)*cos(theta_t), sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t), sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t), 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, sin(psi_t)*cos(theta_t), sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t), -sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t), 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, -sin(theta_t), sin(phi_t)*cos(theta_t), cos(phi_t)*cos(theta_t), 0, 0, 0, 0,
            
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

    Eigen::Matrix<double, n, n> A_d;
    Eigen::Matrix<double, n, m> B_d;

    discretize_state_space_matrices(A_c, B_c, dt, A_d, B_d);

    return A_d * x + B_d * u;
}

void run_mpc() {

    // Setting up UDP server socket...
    
}

int main()
{
    // Initiate damping ratio matrix, desired natural frequency, orientation gains as well as desired trajectory to avoid null pointer
    h << 0.6, 0, 0,
         0, 0.6, 0,
         0, 0, 0.6;
                            
    omega_desired << 10 * M_PI, 16 * M_PI, 10 * M_PI;

    Kp_orientation = 9;
    Kd_orientation = 0.15;

    pos_desired_left_leg << 0, 0, -1.115, 0, 0; //cartesian xyz + euler roll and pitch
    vel_desired_left_leg << 0, 0, 0, 0, 0; //cartesian xyz + euler roll and pitch
    accel_desired_left_leg << 0, 0, 0; //cartesian xyz
    
    //left_leg_state_thread = std::thread(std::bind(update_left_leg_state));
    left_leg_torque_thread = std::thread(std::bind(calculate_left_leg_torques)); // Bind function to thread
    //left_leg_mpc_thread = std::thread(std::bind(run_mpc));

    std::cout << "omega_desired is currently:" << omega_desired(0) << ", " << omega_desired(1) << ", " << omega_desired(2) << std::endl; // Print out current natural frequency
    std::cout << std::endl << std::endl << std::endl;

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

    ipopt_opts["max_iter"] = 40;
    ipopt_opts["print_level"] = 0;
    ipopt_opts["acceptable_tol"] = 1e-7;
    ipopt_opts["acceptable_obj_change_tol"] = 1e-5;

    opts["print_time"] = 1;
    opts["ipopt"] = ipopt_opts;
    opts["expand"] = false;

    Function solver = nlpsol("solver", "ipopt", "../nlp.so", opts);

    std::map<std::string, DM> solver_arguments, solution;
    
    const int num_constraint_bounds = n * (N+1) + m * N + N * 8; // n * (N+1) for initial state and dynamics at each time step, m * N for reaction forces at each time step, N * 8 for friction constraints
    const int num_decision_variable_bounds = n * (N+1) + m*N;
    const int num_decision_variables = n * (N+1) + m * N;

    std::cout << "Constraint Bounds length: " << num_constraint_bounds << std::endl;
    std::cout << "Decision variable bounds length: " << num_decision_variable_bounds << std::endl;
    std::cout << "Decision variables length: " << num_decision_variables << std::endl;
    // DM lbg[constraint_length];
    // DM ubg[constraint_length];

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

    Eigen::Matrix<double, n, 1> x_t = (Eigen::Matrix<double, n, 1>() << 0, 0, 0, 0, 0, 1.48, 0, 0, 0, 0, 0, 0, -9.81).finished();
    Eigen::Matrix<double, m, 1> u_t = Eigen::ArrayXXd::Zero(m, 1);

    Eigen::Matrix<double, n*(N+1)+m*N, 1> x0_solver = Eigen::ArrayXXd::Zero(n*(N+1)+m*N, 1);
    Eigen::Matrix<double, n*(N+1), 1> X_t = Eigen::ArrayXXd::Zero(n*(N+1), 1); // Maybe this is actually obsolete and only x0_solver is sufficient
    Eigen::Matrix<double, m*N, 1> U_t = Eigen::ArrayXXd::Zero(m*N, 1); // Same here

    Eigen::Matrix<double, n, N> x_ref = Eigen::ArrayXXd::Zero(n, N);;

    static const int P_rows = n;
    static const int P_cols = 1 + N + n * N + m * N + N * m;
    Eigen::Matrix<double, P_rows, P_cols> P_param = Eigen::ArrayXXd::Zero(P_rows, P_cols);

    Eigen::Matrix<double, n , n> A_d_array[N];
    Eigen::Matrix<double, n, m> B_d_array[N];

    Eigen::Matrix<double, 3, 3> I_world = Eigen::ArrayXXd::Zero(3, 3);

    long long total_iterations = 0;

    double pos_x_desired = 0;
    double pos_y_desired = 0.0;
    double pos_z_desired = 1.5;

    double vel_x_desired = 0.0;
    double vel_y_desired = 0.0;
    double vel_z_desired = 0.0;

    double phi_desired = 0;
    double theta_desired = 0;
    double psi_desired = 0;

    double omega_x_desired = 0;
    double omega_y_desired = 0;
    double omega_z_desired = 0;

    static const int contact_swap_interval = 15;
    double t_stance = contact_swap_interval * dt;
    double gait_gain = 0.1; // Rename to more accurate name

    double r_x_limit = 0.5; // The relative foot position in x (so r_x_left and r_x_right) is limited to +/- r_x_limit

    double r_x_left = 0;
    double r_y_left = 0;
    double r_z_left = 0;

    double r_x_right = 0;
    double r_y_right = 0;
    double r_z_right = 0;

    Eigen::Matrix<double, 3, 1> left_foot_pos_world = Eigen::ArrayXd::Zero(3, 1);
    Eigen::Matrix<double, 3, 1> right_foot_pos_world = Eigen::ArrayXd::Zero(3, 1);

    Eigen::Matrix<double, 3, 1> left_foot_pos_world_prev = Eigen::ArrayXd::Zero(3, 1);
    Eigen::Matrix<double, 3, 1> right_foot_pos_world_prev = Eigen::ArrayXd::Zero(3, 1);

    double hip_offset = 0.15;
    
    // These might need renaming in future since it might cause problems when accessing from other threads while discretizing at future values
    // double phi_t = 0;
    // double theta_t = 0;
    // double psi_t = 0;

    Eigen::Matrix<double, n, n> A_c = Eigen::ArrayXXd::Zero(n, n);
    Eigen::Matrix<double, n, m> B_c = Eigen::ArrayXXd::Zero(n, m);
    
    Eigen::Matrix<double, 3, 3> r_left_skew_symmetric = Eigen::ArrayXXd::Zero(3, 3);
    Eigen::Matrix<double, 3, 3> r_right_skew_symmetric = Eigen::ArrayXXd::Zero(3, 3);

    Eigen::Matrix<double, m, m*N> D_vector = Eigen::ArrayXXd::Zero(m, m*N);
    Eigen::Matrix<double, m, m> D_k = Eigen::ArrayXXd::Zero(m, m);

    bool swing_left = true; // Still have to figure out how to change between standing in place and stepping / walking
    bool swing_right = false;

    solver_arguments["lbg"] = lbg;
    solver_arguments["ubg"] = ubg;
    solver_arguments["lbx"] = lbx;
    solver_arguments["ubx"] = ubx;

    ofstream data_file;
    data_file.open(".././plot_data/mpc_log.csv");
    data_file << "t,phi,theta,psi,pos_x,pos_y,pos_z,omega_x,omega_y,omega_z,vel_x,vel_y,vel_z,g,f_x_left,f_y_left,f_z_left,f_x_right,f_y_right,f_z_right,r_x_left,r_y_left,r_z_left,r_x_right,r_y_right,r_z_right" << std::endl; // Add header to csv file
    data_file.close();

    struct timespec deadline; // timespec struct for storing time that execution thread should sleep for

    while(true) {
        // Loop starts here
        auto start = high_resolution_clock::now();
        auto start_total = high_resolution_clock::now();

        //msg_length = recvfrom(sockfd, (char *)buffer, udp_buffer_size, 0, ( struct sockaddr *) &cliaddr, &len); // Receive message over UDP containing full leg state
        //buffer[msg_length] = '\0'; // Add string ending delimiter to end of string (n is length of message)
        
        //std::string raw_state(buffer); // Create string from buffer char array to split

        //std::vector<std::string> com_state = split_string(raw_state, '|'); // Split raw state message by message delimiter to parse individual elements

        for(int i = 0; i < n; ++i) {
            //x_t(i, 0) = atof(com_state[i].c_str());
            P_param(i,0) = x_t(i, 0);
        }

        // = x_t // Add delay compensation by stepping system here, use function to make code more compact.

        std::cout << "x_t:" << x_t(0, 0) << "," << x_t(1, 0) << "," << x_t(2, 0) << "," << x_t(3, 0) << "," << x_t(4, 0) << "," << x_t(5, 0) << "," << x_t(6, 0) << "," << x_t(7, 0) << "," << x_t(8, 0) << "," << x_t(9, 0) << "," << x_t(10, 0) << "," << x_t(11, 0) << "," << x_t(12, 0) << std::endl;

        if (total_iterations % contact_swap_interval == 0) {
            swing_left = !swing_left;
            swing_right = !swing_right;
        }

        bool swing_left_temp = swing_left;
        bool swing_right_temp = swing_right;

        for(int k = 0; k < N; ++k) {
            if((total_iterations + k) % contact_swap_interval == 0 && k != 0) {
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

        P_param.block<m, m*N> (0, 1 + N + n*N + m*N) = D_vector;

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
            Eigen::Matrix<double, 3,1> adjusted_pos_vector_left = (Eigen::Matrix<double, 3,1>() << x_t(3, 0) - hip_offset, x_t(4, 0), x_t(5, 0)).finished();
            Eigen::Matrix<double, 3, 1> adjusted_pos_vector_right = (Eigen::Matrix<double, 3,1>() << x_t(3, 0) + hip_offset, x_t(4, 0), x_t(5, 0)).finished();
            
            Eigen::Matrix<double, 3, 1> vel_vector = (Eigen::Matrix<double, 3, 1>() << x_t(9, 0), x_t(10, 0), x_t(11, 0)).finished();
            Eigen::Matrix<double, 3, 1> vel_desired_vector = (Eigen::Matrix<double, 3, 1>() << vel_x_desired, vel_y_desired, vel_z_desired).finished();
            Eigen::Matrix<double, 3, 1> omega_desired_vector = (Eigen::Matrix<double, 3, 1>() << omega_x_desired, omega_y_desired, omega_z_desired).finished();

            left_foot_pos_world = adjusted_pos_vector_left /*+ (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(x_t(5, 0)) / 9.81) * vel_vector.cross(omega_desired_vector)*/;
            right_foot_pos_world = adjusted_pos_vector_right /*+ (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(x_t(5, 0)) / 9.81) * vel_vector.cross(omega_desired_vector)*/;
            
            if (left_foot_pos_world(0, 0) - x_t(3, 0) > r_x_limit) {
                left_foot_pos_world(0, 0) = x_t(3, 0) + r_x_limit;
            }
            else if (left_foot_pos_world(0, 0) - x_t(3, 0) < -r_x_limit) {
                left_foot_pos_world(0, 0) = x_t(3, 0) - r_x_limit;
            }

            if (right_foot_pos_world(0, 0) - x_t(3, 0) > r_x_limit) {
                right_foot_pos_world(0, 0) = x_t(3, 0) + r_x_limit;
            }
            else if (right_foot_pos_world(0, 0) - x_t(3, 0) < -r_x_limit) {
                right_foot_pos_world(0, 0) = x_t(3, 0) - r_x_limit;
            }
        }

        r_x_left = left_foot_pos_world(0, 0) - x_t(3, 0);
        r_x_right = right_foot_pos_world(0, 0) - x_t(3, 0);

        r_y_left = left_foot_pos_world(1, 0) - x_t(4, 0);
        r_y_right = right_foot_pos_world(1, 0) - x_t(4, 0);

        // r_x_left = -hip_offset;
        // r_x_right = hip_offset;
        // r_y_left = r_y_right = 0;

        r_z_left = -x_t(5, 0);
        r_z_right = -x_t(5, 0);

        static double pos_x_desired_temp = pos_x_desired;
        static double pos_y_desired_temp = pos_y_desired;
        static double pos_z_desired_temp = pos_z_desired;

        static double phi_desired_temp = phi_desired;
        static double theta_desired_temp = theta_desired;
        static double psi_desired_temp = psi_desired;

        for(int i = 0; i < N; ++i) {
            pos_x_desired_temp += vel_x_desired * dt;
            pos_y_desired_temp += vel_y_desired * dt;
            pos_z_desired_temp += vel_z_desired * dt;

            phi_desired_temp += omega_x_desired * dt;
            theta_desired_temp += omega_y_desired * dt;
            psi_desired_temp += omega_z_desired * dt;

            x_ref(0, i) = phi_desired_temp; // Roll
            x_ref(1, i) = theta_desired_temp; // Pitch
            x_ref(2, i) = psi_desired_temp; // Yaw
            x_ref(3, i) = pos_x_desired_temp; // X Pos
            x_ref(4, i) = pos_y_desired_temp; // Y Pos
            x_ref(5, i) = pos_z_desired_temp; // Z Pos
            x_ref(6, i) = omega_x_desired; // Omega_x
            x_ref(7, i) = omega_y_desired; // Omega_y
            x_ref(8, i) = omega_z_desired; // Omega_z
            x_ref(9, i) = vel_x_desired; // X Vel
            x_ref(10, i) = vel_y_desired; // Y Vel
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

        static double r_y_left_prev = r_y_left;
        static double r_y_right_prev = r_y_right;

        static double r_x_left_prev = r_x_left;
        static double r_x_right_prev = r_x_right;

        static double r_z_left_prev = r_z_left;
        static double r_z_right_prev = r_z_right;

        Eigen::MatrixXd left_foot_pos_world_prev = left_foot_pos_world;
        Eigen::MatrixXd right_foot_pos_world_prev = right_foot_pos_world;

        static double phi_t = 0.0;
        static double theta_t = 0.0;
        static double psi_t = 0.0;

        static double vel_x_t = 0.0;
        static double vel_y_t = 0.0;
        static double vel_z_t = 0.0;
        
        static double pos_x_t = 0.0;
        static double pos_y_t = 0.0;
        static double pos_y_t_next = 0.0;
        static double pos_z_t = 0.0;

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
                phi_t = x_t(0, 0);
                theta_t = x_t(1, 0);
                psi_t = x_t(2, 0);

                vel_x_t = x_t(9, 0);
                vel_y_t = x_t(10, 0);
                vel_z_t = x_t(11, 0);

                pos_x_t = x_t(3, 0);
                pos_y_t = x_t(4, 0);
                pos_z_t = x_t(5, 0);
            }

            if((total_iterations+i) % contact_swap_interval == 0 && i != 0) {
                Eigen::Matrix<double, 3, 1> adjusted_pos_vector_left = (Eigen::Matrix<double, 3,1>() << pos_x_t - hip_offset, pos_y_t, pos_z_t).finished();
                Eigen::Matrix<double, 3, 1> adjusted_pos_vector_right = (Eigen::Matrix<double, 3,1>() << pos_x_t + hip_offset, pos_y_t, pos_z_t).finished();
                
                Eigen::Matrix<double, 3, 1> vel_vector = (Eigen::Matrix<double, 3, 1>() << vel_x_t, vel_y_t, vel_z_t).finished();
                Eigen::Matrix<double, 3, 1> vel_desired_vector = (Eigen::Matrix<double, 3, 1>() << vel_x_desired, vel_y_desired, vel_z_desired).finished();
                Eigen::Matrix<double, 3, 1> omega_desired_vector = (Eigen::Matrix<double, 3, 1>() << omega_x_desired, omega_y_desired, omega_z_desired).finished();

                left_foot_pos_world = adjusted_pos_vector_left /*+ (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(pos_z_t) / 9.81) * vel_vector.cross(omega_desired_vector)*/;
                right_foot_pos_world = adjusted_pos_vector_right /*+ (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) /*+ 0.5 * sqrt(abs(pos_z_t) / 9.81) * vel_vector.cross(omega_desired_vector)*/;
                
                if (left_foot_pos_world(0, 0) - pos_x_t > r_x_limit) {
                    left_foot_pos_world(0, 0) = pos_x_t + r_x_limit;
                }
                else if (left_foot_pos_world(0, 0) - pos_x_t < -r_x_limit) {
                    left_foot_pos_world(0, 0) = pos_x_t - r_x_limit;
                }

                if (right_foot_pos_world(0, 0) - pos_x_t > r_x_limit) {
                    right_foot_pos_world(0, 0) = pos_x_t + r_x_limit;
                }
                else if (right_foot_pos_world(0, 0) - pos_x_t < -r_x_limit) {
                    right_foot_pos_world(0, 0) = pos_x_t - r_x_limit;
                }
            }

            r_x_left = left_foot_pos_world(0, 0) - pos_x_t;
            r_x_right = left_foot_pos_world(0, 0) - pos_x_t;

            r_y_left = left_foot_pos_world(0, 0) - pos_y_t;
            r_y_right = right_foot_pos_world(0, 0) - pos_y_t;

            // r_x_left = -hip_offset;
            // r_x_right = hip_offset;

            // r_y_left = r_y_right = 0;

            r_z_left = -pos_z_t;
            r_z_right = -pos_z_t;

            I_world << -(Ixy*sin(theta_t) + Ixy*cos(psi_t)*cos(theta_t) - Iyy*sin(psi_t)*cos(theta_t))*sin(psi_t)*cos(theta_t) + (Ixx*cos(psi_t)*cos(theta_t) - Iyx*sin(psi_t)*cos(theta_t) + Izx*sin(theta_t))*cos(psi_t)*cos(theta_t) + (Ixz*cos(psi_t)*cos(theta_t) - Iyz*sin(psi_t)*cos(theta_t) + Izz*sin(theta_t))*sin(theta_t), (-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*sin(theta_t) + Ixy*cos(psi_t)*cos(theta_t) - Iyy*sin(psi_t)*cos(theta_t)) + (sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t))*(Ixx*cos(psi_t)*cos(theta_t) - Iyx*sin(psi_t)*cos(theta_t) + Izx*sin(theta_t)) - (Ixz*cos(psi_t)*cos(theta_t) - Iyz*sin(psi_t)*cos(theta_t) + Izz*sin(theta_t))*sin(phi_t)*cos(theta_t), (sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixx*cos(psi_t)*cos(theta_t) - Iyx*sin(psi_t)*cos(theta_t) + Izx*sin(theta_t)) + (sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixy*sin(theta_t) + Ixy*cos(psi_t)*cos(theta_t) - Iyy*sin(psi_t)*cos(theta_t)) + (Ixz*cos(psi_t)*cos(theta_t) - Iyz*sin(psi_t)*cos(theta_t) + Izz*sin(theta_t))*cos(phi_t)*cos(theta_t),
                        (Ixx*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyx*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izx*sin(phi_t)*cos(theta_t))*cos(psi_t)*cos(theta_t) - (Ixy*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) - Ixy*sin(phi_t)*cos(theta_t) + Iyy*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)))*sin(psi_t)*cos(theta_t) + (Ixz*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyz*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izz*sin(phi_t)*cos(theta_t))*sin(theta_t), (-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) - Ixy*sin(phi_t)*cos(theta_t) + Iyy*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))) + (sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t))*(Ixx*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyx*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izx*sin(phi_t)*cos(theta_t)) - (Ixz*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyz*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izz*sin(phi_t)*cos(theta_t))*sin(phi_t)*cos(theta_t), (sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixx*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyx*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izx*sin(phi_t)*cos(theta_t)) + (sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixy*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) - Ixy*sin(phi_t)*cos(theta_t) + Iyy*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))) + (Ixz*(sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t)) + Iyz*(-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t)) - Izz*sin(phi_t)*cos(theta_t))*cos(phi_t)*cos(theta_t), 
                        (Ixx*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyx*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izx*cos(phi_t)*cos(theta_t))*cos(psi_t)*cos(theta_t) - (Ixy*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)))*sin(psi_t)*cos(theta_t) + (Ixz*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyz*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izz*cos(phi_t)*cos(theta_t))*sin(theta_t), (-sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t))*(Ixy*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))) + (sin(phi_t)*sin(theta_t)*cos(psi_t) + sin(psi_t)*cos(phi_t))*(Ixx*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyx*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izx*cos(phi_t)*cos(theta_t)) - (Ixz*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyz*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izz*cos(phi_t)*cos(theta_t))*sin(phi_t)*cos(theta_t), (sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t))*(Ixx*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyx*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izx*cos(phi_t)*cos(theta_t)) + (sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))*(Ixy*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Ixy*cos(phi_t)*cos(theta_t) + Iyy*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t))) + (Ixz*(sin(phi_t)*sin(psi_t) - sin(theta_t)*cos(phi_t)*cos(psi_t)) + Iyz*(sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t)) + Izz*cos(phi_t)*cos(theta_t))*cos(phi_t)*cos(theta_t);

            r_left_skew_symmetric << 0, -r_z_left, r_y_left,
                                        r_z_left, 0, -r_x_left,
                                        -r_y_left, r_x_left, 0;
                    
            r_right_skew_symmetric << 0, -r_z_right, r_y_right,
                                        r_z_right, 0, -r_x_right,
                                        -r_y_right, r_x_right, 0;

            A_c << 0, 0, 0, 0, 0, 0, cos(psi_t)*cos(theta_t), sin(phi_t)*sin(theta_t)*cos(psi_t) - sin(psi_t)*cos(phi_t), sin(phi_t)*sin(psi_t) + sin(theta_t)*cos(phi_t)*cos(psi_t), 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, sin(psi_t)*cos(theta_t), sin(phi_t)*sin(psi_t)*sin(theta_t) + cos(phi_t)*cos(psi_t), -sin(phi_t)*cos(psi_t) + sin(psi_t)*sin(theta_t)*cos(phi_t), 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, -sin(theta_t), sin(phi_t)*cos(theta_t), cos(phi_t)*cos(theta_t), 0, 0, 0, 0,
                    
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

            discretize_state_space_matrices(A_c, B_c, dt, A_d_t, B_d_t);

            //A_d_array[i] = A_d_t;
            //B_d_array[i] = B_d_t;

            P_param.block<n, n>(0, 1 + N + (i*n)) = A_d_t;
            P_param.block<n, m>(0, 1 + N + n * N + (i*m)) = B_d_t;
        }

        left_foot_pos_world = left_foot_pos_world_prev;
        right_foot_pos_world = right_foot_pos_world_prev;

        r_x_left = r_x_left_prev;
        r_x_right = r_x_right_prev;

        r_y_left = r_y_left_prev;
        r_y_right = r_y_right_prev;

        r_z_left = r_z_right = -x_t(5, 0);

        size_t rows_P_param = P_param.rows();
        size_t cols_P_param = P_param.cols();

        DM P_param_casadi = casadi::DM::zeros(rows_P_param, cols_P_param);

        std::memcpy(P_param_casadi.ptr(), P_param.data(), sizeof(double)*rows_P_param*cols_P_param);
        
        solver_arguments["p"] = P_param_casadi;

        // std::cout << "P_param_casadi matrices (B):\n" << std::endl;
        // std::cout << P_param_casadi(Slice(0, n), Slice(1 + N + n*N, 1 + N + n*N + m)) << std::endl;
        // std::cout << "\nx_t and x_ref in P_Param_casadi:\n" << std::endl;
        // std::cout << P_param_casadi(Slice(0, n), Slice(0, N+1)) << std::endl;
        // std::cout << "D_vector in P_Param_casadi:\n" << P_param_casadi(Slice(0, n), Slice(1+N+n*N+m*N, 1+N+n*N+m*N+m*N)) << std::endl;
        // std::cout << std::endl;
        // std::cout << P_param_casadi(Slice(0, n), Slice(1 + N + n*N + 3*m, 1 + N + n*N + 3*m + m)) << std::endl;
        
        x0_solver << X_t, U_t;

        size_t rows_x0_solver = x0_solver.rows();
        size_t cols_x0_solver = x0_solver.cols();

        DM x0_solver_casadi = casadi::DM::zeros(rows_x0_solver, cols_x0_solver);

        std::memcpy(x0_solver_casadi.ptr(), x0_solver.data(), sizeof(double)*rows_x0_solver*cols_x0_solver);
        solver_arguments["x0"] = x0_solver_casadi;

        auto end = high_resolution_clock::now();

        double duration_before = duration_cast<microseconds>(end - start).count();

        auto sol_start = high_resolution_clock::now();

        solution = solver(solver_arguments);

        auto sol_end = high_resolution_clock::now();
        double solver_time = duration_cast<microseconds>(sol_end - sol_start).count();

        start = high_resolution_clock::now();

        size_t rows = solution.at("x").size1();
        size_t cols = solution.at("x").size2();

        Eigen::Matrix<double, n*(N+1) + m*N, 1> solution_variables = Eigen::ArrayXXd::Zero(n*(N+1) + m*N, 1);

        solution_variables.resize(rows,cols);
        solution_variables.setZero(rows,cols);

        std::memcpy(solution_variables.data(), solution.at("x").ptr(), sizeof(double)*rows*cols);

        u_t << solution_variables(n*(N+1)+0),
                solution_variables(n*(N+1)+1),
                solution_variables(n*(N+1)+2),
                solution_variables(n*(N+1)+3),
                solution_variables(n*(N+1)+4),
                solution_variables(n*(N+1)+5);
        
        x_t = step_discrete_model(x_t, u_t, r_x_left, r_x_right, r_y_left, r_y_right, r_z_left, r_z_right);

        std::cout << "after stepping model" << std::endl;

        stringstream s;

        // s << u_t(0) << "|" << u_t(1) << "|" << u_t(2) << "|" << u_t(3) << "|" << u_t(4) << "|" << u_t(5) << "|" << r_x_left << "|" << r_y_left << "|" << r_z_left << "|" << r_x_right << "|" << r_y_right << "|" << r_z_right; // Write torque setpoints to stringstream

        // sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len);
        
        ofstream data_file;
        data_file.open(".././plot_data/mpc_log.csv", ios::app); // Open csv file in append mode
        data_file << total_iterations * dt << "," << x_t(0, 0) << "," << x_t(1, 0) << "," << x_t(2, 0) << "," << x_t(3, 0) << "," << x_t(4, 0) << "," << x_t(5, 0) << "," << x_t(6, 0) << "," << x_t(7, 0) << "," << x_t(8, 0) << "," << x_t(9, 0) << "," << x_t(10, 0) << "," << x_t(11, 0) << "," << x_t(12, 0)
                << "," << u_t(0) << "," << u_t(1) << "," << u_t(2) << "," << u_t(3) << "," << u_t(4) << "," << u_t(5) 
                << "," << r_x_left << "," << r_y_left << "," << r_z_left << "," << r_x_right << "," << r_y_right << "," << r_z_right << ",0" << std::endl; // Zero at the end has to be replace with predicted delay compensation state!
        data_file.close(); // Close csv file again. This way thread abort should (almost) never leave file open.

        X_t.block<n*N, 1>(0, 0) = solution_variables.block<n*N, 1>(n, 0);
        X_t.block<n, 1>(n*N, 0) = solution_variables.block<n, 1>(n*N, 0);

        U_t.block<m*(N-1), 1>(0, 0) = solution_variables.block<m*(N-1), 1>(n*(N+1) + m, 0);
        U_t.block<m, 1>(m*(N-1), 0) = solution_variables.block<m, 1>(n*(N+1)+m*(N-1), 0);

        ++total_iterations;

        end = high_resolution_clock::now();
        double duration_after = duration_cast<microseconds> (end - start).count();

        std::cout << "Solver preparation took " << duration_before + duration_after << " microseconds" << std::endl;

        std::cout << "u_t: " << u_t(0) << "," << u_t(1) << "," << u_t(2) << "," << u_t(3) << "," << u_t(4) << "," << u_t(5) << std::endl;

        auto end_total = high_resolution_clock::now();
        double full_iteration_duration = duration_cast<microseconds> (end_total - start_total).count();

        std::cout << "Full iteration took " << full_iteration_duration << " microseconds" << std::endl;

        long long remainder = (dt * 1e+6 - full_iteration_duration) * 1e+3;
        //std::cout << "Remainder: " << remainder << " microseconds" << std::endl;
        deadline.tv_nsec = remainder;
        deadline.tv_sec = 0;
        clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
    }

    while(true) {

    }
}