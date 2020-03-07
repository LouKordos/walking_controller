#include <iostream>

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
#include <sys/types.h>

#include "leg_state.hpp"
#include "torque_setpoint.hpp"

#include <errno.h> //It defines macros for reporting and retrieving error conditions through error codes
#include <time.h> //contains various functions for manipulating date and time
#include <unistd.h> //contains various constants
#include <sys/types.h> //contains a number of basic derived types that should be used whenever appropriate
#include <arpa/inet.h> // defines in_addr structure
#include <sys/socket.h> // for socket creation
#include <netinet/in.h> //contains constants and structures needed for internet domain addresses

#include "model_functions.cpp"

using namespace std;
using namespace std::chrono;

//zcm::ZCM zcm_context { "ipc" };

const char* left_leg_state_channel = ".left_leg_state";
const char* right_leg_state_channel = ".right_leg_state";

const char* left_leg_torque_setpoint_channel = "left_leg_torque_setpoint";
const char* right_leg_torque_setpoint_channel = "right_leg_torque_setpoint";

const int udp_port = 4200;

const int udp_buffer_size = 4096;

std::vector<std::string> split_string(std::string str, char delimiter) {
    std::vector<std::string> results;

    boost::split(results, str, [&delimiter](char c){return c == delimiter;});

    return results;
}

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


std::thread left_leg_state_thread;
std::thread right_leg_state_thread;

std::thread left_leg_torque_thread;
std::thread right_leg_torque_thread;

double state_update_interval = 1000.0; //microseconds
double torque_calculation_interval = 1000.0; //microseconds

void update_left_leg_state() {

    high_resolution_clock::time_point start = high_resolution_clock::now();
    high_resolution_clock::time_point end = high_resolution_clock::now();

    double duration = 0.0f;

    struct timespec deadline;

    while(true) {
        // This timed loop approach calculates the time that the execution of the main code took,
        // then calculates the remaining time for the loop and waits this duration.
        start = high_resolution_clock::now();
        //Do stuff
        end = high_resolution_clock::now();
        duration = duration_cast<microseconds>(end - start).count();  
        long long remainder = (state_update_interval - duration) * 1e+03;
        deadline.tv_nsec = remainder;
        deadline.tv_sec = 0;
        clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
    }
}

void calculate_left_leg_torques() {
    high_resolution_clock::time_point start = high_resolution_clock::now();
    high_resolution_clock::time_point end = high_resolution_clock::now();

    struct timespec deadline;

    double duration = 0.0f;

    Eigen::Matrix<double, 5, 1> q_temp;
    Eigen::Matrix<double, 5, 1> q_dot_temp;

    long long iteration_counter = 0;
    float dt = torque_calculation_interval / 1000 / 1000; //in seconds

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
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE);
    } 
      
    int n; 

    socklen_t len;


    time_t now = time(0);
    tm *localTime = localtime(&now);

    std::string filename = std::to_string(localTime->tm_mday) + "-" + std::to_string(1 + localTime->tm_mon) + "-" + std::to_string(localTime->tm_year + 1900) 
                    + "-" + std::to_string(localTime->tm_hour + 1) + ":" + std::to_string(localTime->tm_min + 1) + ":" + std::to_string(localTime->tm_sec + 1);

    ofstream data_file;
    data_file.open("/home/loukas/Documents/cpp/walking_controller/plot_data/data_" + filename + ".csv");
    data_file << "theta1,theta2,theta3,theta4,theta5,theta1_dot,theta2_dot,theta3_dot,theta4_dot,theta5_dot," 
            << "G_1,G_2,G_3,G_4,G_5,"
            << "Kp_diag_1,Kp_diag_2,Kp_diag_3,"
            << "Kd_diag_1,Kd_diag_2,Kd_diag_3,"
            << "foot_pos_x,foot_pos_y,foot_pos_z,"
            << "foot_vel_x,foot_vel_y,foot_vel_z,"
            << "tau_1,tau_2,tau_3,tau_4,tau_5" << std::endl;
    data_file.close();

    while(true) {
        start = high_resolution_clock::now();

        double t = iteration_counter * dt;

        pos_desired_left_leg << 0, 0, 0.1*sin(2*t) - 0.9, 0, 0;

        vel_desired_left_leg << 0, 0, 0.2*cos(2*t), 0, 0;

        accel_desired_left_leg << 0, 0, -0.4*sin(2*t);
        
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

        n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);
        buffer[n] = '\0';

        string raw_state(buffer);

        std::vector<std::string> state = split_string(raw_state, '|');

        if(static_cast<int>(state.size()) >= 9) {

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

            q_temp << theta1, theta2, theta3, theta4, theta5;
            q_dot_temp << theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot;
        }

        update_orientation(theta1, theta2, theta3, theta4, theta5);

        update_B_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

        update_J_foot_left_leg(theta1, theta2, theta3, theta4, theta5);

        update_J_foot_combined_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

        update_J_foot_dot_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

        //Set singular Jacobians to zero

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

        update_tau_ff_left_leg(q_dot_temp);

        update_Kp_left_leg();
        update_Kd_left_leg();

        update_foot_pos_left_leg(theta1, theta2, theta3, theta4, theta5);

        update_foot_vel_left_leg(q_dot_temp);

        update_tau_setpoint_left_leg();

        for(int i = 0; i < 5; ++i) {
            constrain(tau_setpoint_left_leg(i), lower_torque_limit, upper_torque_limit);
        }
        //tau_setpoint_left_leg(4) = 0;

        torque_setpoint setpoint;

        setpoint.tau1 = tau_setpoint_left_leg(0);
        setpoint.tau2 = tau_setpoint_left_leg(1);
        setpoint.tau3 = tau_setpoint_left_leg(2);
        setpoint.tau4 = tau_setpoint_left_leg(3);
        setpoint.tau5 = tau_setpoint_left_leg(4);

        ofstream data_file;
        data_file.open("/home/loukas/Documents/cpp/walking_controller/plot_data/data_" + filename + ".csv", ios::app);
        data_file << theta1 << "," << theta2 << "," << theta3 << "," << theta4 << "," << theta5 
            << "," << theta1_dot << "," << theta2_dot << "," << theta3_dot << "," << theta4_dot << "," << theta5_dot 
            << "," << G_left_leg(0) << "," << G_left_leg(1) << "," << G_left_leg(2) << "," << G_left_leg(3) << "," << G_left_leg(4)
            << "," << Kp_left_leg(0, 0) << "," << Kp_left_leg(1, 1) << "," << Kp_left_leg(2, 2)
            << "," << Kd_left_leg(0, 0) << "," << Kd_left_leg(1, 1) << "," << Kd_left_leg(2, 2)
            << "," << foot_pos_left_leg(0) << "," << foot_pos_left_leg(1) << "," << foot_pos_left_leg(2)
            << "," << foot_vel_left_leg(0) << "," << foot_vel_left_leg(1) << "," << foot_vel_left_leg(2)
            << "," << setpoint.tau1 << "," << setpoint.tau2 << "," << setpoint.tau3 << "," << setpoint.tau4 << "," << setpoint.tau5 << std::endl;
        data_file.close();

        std::cout << "roll: " << phi << ", pitch: " << theta << ", yaw: " << psi << std::endl;

        std::string filename = std::to_string(localTime->tm_mday) + "-" + std::to_string(1 + localTime->tm_mon) + "-" + std::to_string(localTime->tm_year + 1900) 
                        + "-" + std::to_string(localTime->tm_hour + 1) + ":" + std::to_string(localTime->tm_min + 1) + ":" + std::to_string(localTime->tm_sec + 1);

        stringstream s;

        s << setpoint.tau1 << "|" << setpoint.tau2 << "|" << setpoint.tau3 << "|" << setpoint.tau4 << "|" << setpoint.tau5;

        sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len);

        iteration_counter++;

        end = high_resolution_clock::now();

        duration = duration_cast<microseconds>(end - start).count();
        // std::cout << "Loop duration: " << duration << "µS, iteration_counter: " << iteration_counter - 1 << std::endl;
        long long remainder = (torque_calculation_interval - duration) * 1e+03;
        deadline.tv_nsec = remainder;
        deadline.tv_sec = 0;
        clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
    }
}

int main()
{
    h << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
                            
    omega_desired << 19 * M_PI, 19 * M_PI, 19 * M_PI;

    Kp_orientation = 2;
    Kd_orientation = 0.25;

    pos_desired_left_leg << 0, 0, -1.115, 0, 0; //cartesian xyz + euler roll and pitch
    vel_desired_left_leg << 0, 0, 0, 0, 0; //cartesian xyz + euler roll and pitch
    accel_desired_left_leg << 0, 0, 0; //cartesian xyz
    
    //left_leg_state_thread = std::thread(std::bind(update_left_leg_state));
    //right_leg_state_thread = std::thread(std::bind(update_right_leg_state));
    left_leg_torque_thread = std::thread(std::bind(calculate_left_leg_torques));
    //right_leg_torque_thread = std::thread(std::bind(calculate_right_leg_torques));

    std::cout << "omega_desired is currently:" << omega_desired(0) << ", " << omega_desired(1) << ", " << omega_desired(2) << std::endl;

    //std::cout << B_left_leg << std::endl;

    while(true) {

    }
}
//COMPILE: g++ controller.cpp leg_state.hpp torque_setpoint.hpp -o controller -lzcm -pthread -I .
//Also, ALWAYS run as root due to IPC rights if using ZCM