#include <iostream>
#include <dirent.h>

#include <chrono>
#include <thread>
#include <functional>

#include <fstream>

#include "nameof.h"

#include <string>
#include <random>
#include <ctime>
#include <cmath>

#include <fstream>

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
#include <unsupported/Eigen/MatrixFunctions>

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
    servaddr.sin_addr.s_addr = INADDR_ANY; 
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
        perror ("");
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

        n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len); // Receive message over UDP containing full leg state
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

void discretize_state_space_matrices(const Eigen::MatrixXd &A_c, const Eigen::MatrixXd &B_c, const double &dt, Eigen::MatrixXd &A_d, Eigen::MatrixXd &B_d) {
    static const int n = A_c.rows();
    static const int m = B_c.cols();
    static Eigen::MatrixXd A_B(n + m, n + m);

    A_B << A_c, B_c, Eigen::ArrayXXd::Zero(m, n), Eigen::ArrayXXd::Zero(m, m);
    static const Eigen::MatrixXd e_A_B = (A_B * dt).exp();

    //std::cout << "A_B:\n" << A_B << std::endl;

    A_d = e_A_B.block(0, 0, n, n);
    B_d = e_A_B.block(0, n, n, m);
}

void run_mpc() {

}

int main()
{
    // Initiate damping ratio matrix, desired natural frequency, orientation gains as well as desired trajectory to avoid nulll pointer
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
    left_leg_mpc_thread = std::thread(std::bind(run_mpc));

    std::cout << "omega_desired is currently:" << omega_desired(0) << ", " << omega_desired(1) << ", " << omega_desired(2) << std::endl; // Print out current natural frequency
    std::cout << std::endl << std::endl << std::endl;
    //std::cout << B_left_leg << std::endl;

    DMVector x_t = {0.3, 0.3, 0.3, 0, 0, 1, 0, 0, 0, 0, 0, 0, -9.81};

    DMVector x_ref = {0, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0, -9.81};

    double m_value = 30; // kg

    // For now, the desired state is constant, when that will change, the average angle will have to be calculated from the time-varying state trajectory.
    double avg_psi = ((double)x_ref[2] + (double)x_t[2](0)) / 2.0;

    static const Eigen::Matrix<double, 3, 3> I_world = (Eigen::Matrix<double, 3, 3>() << 0.05, 0.01, 0.01,
                                                                                        0.01, 0.4, 0.01,
                                                                                        0.01, 0.01, 0.03).finished();

    // Location of the force vector being applied by the left foot.
    double r_x_left = 0.1;
    double r_y_left = 0;
    double r_z_left = 0;

    // Location of the force vector being applied by the right foot.
    double r_x_right = -0.1;
    double r_y_right = 0;
    double r_z_right = 0;

    static const int n = 13;
    static const int m = 6;

    static const Eigen::Matrix<double, 3, 3> r_left_skew_symmetric = (Eigen::Matrix<double, 3, 3>() << 0, -r_z_left, r_y_left,
                                                                                            r_z_left, 0, -r_x_left,
                                                                                            -r_y_left, r_x_left, 0).finished(); 
            
    static const Eigen::Matrix<double, 3, 3> r_right_skew_symmetric = (Eigen::Matrix<double, 3, 3>() << 0, -r_z_right, r_y_right,
                                                                                          r_z_right, 0, -r_x_right,
                                                                                          -r_y_right, r_x_right, 0).finished();

    static const Eigen::Matrix<double, n, n> A_c = (Eigen::Matrix<double, n, n>() << 0, 0, 0, 0, 0, 0, cos(avg_psi), sin(avg_psi), 0, 0, 0, 0, 0,
                                                                                        0, 0, 0, 0, 0, 0, -sin(avg_psi), cos(avg_psi), 0, 0, 0, 0, 0,
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                                                                        
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                                                                                        
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                                        
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                                                                                        
                                                                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();

    static const Eigen::Matrix<double, n, m> B_c = (Eigen::Matrix<double, n, m>() << 0, 0, 0, 0, 0, 0,
                                                                                     0, 0, 0, 0, 0, 0,
                                                                                     0, 0, 0, 0, 0, 0,
                                                                                     0, 0, 0, 0, 0, 0,
                                                                                     0, 0, 0, 0, 0, 0,
                                                                                     0, 0, 0, 0, 0, 0,
                                                                                     I_world * r_left_skew_symmetric, I_world * r_right_skew_symmetric,
                                                                                     1/m_value, 0, 0, 1/m_value, 0, 0,
                                                                                     0, 1/m_value, 0, 0, 1/m_value, 0,
                                                                                     0, 0, 1/m_value, 0, 0, 1/m_value,
                                                                                     0, 0, 0, 0, 0, 0).finished();
    double dt = 1/30.0;
    int N = 30;

    double f_min = -250;
    double f_max = 250;

    Function solver = nlpsol("solver", "ipopt", "nlp.so");

    std::map<std::string, DM> solver_arguments, solution;

    // n for initial state, N * n for dynamics at each timestep, (N / m) * m for contact constraints with D * u = 0, (N / m) * 8 for 8 friction constraints.
    int constraints_length = n + N * n + (N / m) * m + (N / m) * 8;
    int bounds_length = n * (N+1) + m * N; // n * (N+1) for initial state and dynamics at each time step, m * N for reaction forces at each time step
    int decision_variables_length = n * (N+1) + m * N;

    std::cout << "Constraints length: " << constraints_length << std::endl;
    std::cout << "Bounds length: " << bounds_length << std::endl;
    std::cout << "Decision variables length: " << decision_variables_length << std::endl;
    // DM lbg[constraint_length];
    // DM ubg[constraint_length];

    DMVector lbg(constraints_length);
    DMVector ubg(constraints_length);

    DMVector lbx(bounds_length);
    DMVector ubx(bounds_length);

    DMVector x0_solver(decision_variables_length);

    // Set 0 for every value besides friction cnstraints (these have to be -inf, take a look at the Point Mass Jupyter Notebook)
    for(int i = 0; i < constraints_length - (N / m) * 8; ++i) { 
        lbg[i] = 0;
    }

    for(int i = constraints_length - (N / m) * 8; i < constraints_length; ++i) {
        lbg[i] = -DM::inf();
    }

    for(int i = 0; i < constraints_length; ++i) {
        ubg[i] = 0;
    }

    for(int i = 0; i < n * (N+1); ++i) {
        lbx[i] = -DM::inf();
        ubx[i] = DM::inf();
    }

    for(int i = n * (N+1); i < bounds_length; ++i) {
        lbx[i] = f_min;
        ubx[i] = f_max;
    }

    std::cout << lbx[n * (N+1)-1] << std::endl;

    DM X_t = DM::repmat(x_t, 1, N+1); // Init with initial state N + 1 times next to each other
    DM U_t = DM::zeros(m, N);
    std::cout << "X_t shape: (" << X_t.rows() << "," << X_t.columns() << ")" << std::endl;
    DMVector test(decision_variables_length);

    // vertcat(*[X_t.reshape((n * (N+1), 1)), U_t.reshape((m * N, 1))])
    test[0] = DM::reshape(X_t, n * (N+1), 1);
    test[1] = DM::reshape(U_t, m * N, 1);
    //std::cout << "X_t:\n" << X_t << std::endl;

    solver_arguments["lbg"] = DM::vertcat(lbg);
    solver_arguments["ubg"] = DM::vertcat(ubg);
    solver_arguments["lbx"] = DM::vertcat(lbx);
    solver_arguments["ubx"] = DM::vertcat(ubx);
    solver_arguments["x0"] = DM::vertcat(test);
    //solver_arguments["x0"] = DM::zeros(583, 1);

    DMVector temp(2*n);
    temp[0] = x_t;
    temp[1] = x_ref;
    solver_arguments["p"] = DM::vertcat(temp);

    solution = solver(solver_arguments);

    while(true) {

    }
}