#include <iostream>

#include <chrono>
#include <thread>
#include <functional>

#include <fstream>

#include <string>
#include <random>
#include <ctime>
#include <cmath>

#include <stdlib.h>
	
#include <boost/algorithm/string.hpp>

#include <unistd.h>
#include <zcm/zcm-cpp.hpp>
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

zcm::ZCM zcm_context { "ipc" };

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

    while(true) {
        start = high_resolution_clock::now();
        
        // q_left_leg_mutex.lock();

        // q_temp = q_left_leg;

        // q_left_leg_mutex.unlock();

        // q_dot_left_leg_mutex.lock();

        // q_dot_temp = q_dot_left_leg;

        // q_dot_left_leg_mutex.unlock();

        // double theta1 = q_temp(0);
        // double theta2 = q_temp(1);
        // double theta3 = q_temp(2);
        // double theta4 = q_temp(3);
        // double theta5 = q_temp(4);

        // //std::cout << q_temp << std::endl;

        // double theta1_dot = q_dot_temp(0);
        // double theta2_dot = q_dot_temp(1);
        // double theta3_dot = q_dot_temp(2);
        // double theta4_dot = q_dot_temp(3);
        // double theta5_dot = q_dot_temp(4);
        
        double theta1;
        double theta2;
        double theta3;
        double theta4;
        double theta5;

        double theta1_dot;
        double theta2_dot;
        double theta3_dot;
        double theta4_dot;
        double theta5_dot;

        n = recvfrom(sockfd, (char *)buffer, udp_buffer_size, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);
        buffer[n] = '\0';

        string raw_state(buffer);

        //std::cout << "Received state: " << raw_state << std::endl;

        std::vector<std::string> state = split_string(raw_state, '|');

        //std::cout << "State size:" << static_cast<int>(state.size()) << std::endl;

        if(static_cast<int>(state.size()) >= 9) {

            theta1 = atof(state[0].c_str());
            theta2 = atof(state[1].c_str());
            theta3 = atof(state[2].c_str());
            theta4 = atof(state[3].c_str());
            theta5 = atof(state[4].c_str());

            //std::cout << "Theta1:" << theta1 << std::endl;

            theta1_dot = atof(state[5].c_str());
            theta2_dot = atof(state[6].c_str());
            theta3_dot = atof(state[7].c_str());
            theta4_dot = atof(state[8].c_str());
            theta5_dot = atof(state[9].c_str());
        }

        // theta1 = 0;
        // theta2 = 0;
        // theta3 = 0;
        // theta4 = 0;
        // theta5 = 0;
        // theta1_dot = 0;
        // theta2_dot = 0;
        // theta3_dot = 0;
        // theta4_dot = 0;
        // theta5_dot = 0;

        update_B_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

        update_J_foot_left_leg(theta1, theta2, theta3, theta4, theta5);

        update_J_foot_dot_left_leg(theta1, theta2, theta3, theta4, theta5, theta1_dot, theta2_dot, theta3_dot, theta4_dot, theta5_dot);

        update_G_left_leg(theta1, theta2, theta3, theta4, theta5);

        //update_C_left_leg();

        update_Lambda_left_leg();

        update_tau_ff_left_leg();

        update_Kp_left_leg();
        update_Kd_left_leg();

        update_foot_pos_left_leg(theta1, theta2, theta3, theta4, theta5);
        std::cout << "Foot position: " << foot_pos_left_leg(0) << "," << foot_pos_left_leg(1) << "," << foot_pos_left_leg(2) << std::endl;
        update_foot_vel_left_leg(q_dot_temp);

        update_tau_setpoint_left_leg();

        for(int i = 0; i < 5; ++i) {
            //tau_setpoint_left_leg(i) = -tau_setpoint_left_leg(i);
            if(isnan(tau_setpoint_left_leg(i))) {
                tau_setpoint_left_leg(i) = 0.1;
            }
            if(tau_setpoint_left_leg(i) > upper_torque_limit) {
                tau_setpoint_left_leg(i) = upper_torque_limit;
            }
            if(tau_setpoint_left_leg(i) < lower_torque_limit) {
                tau_setpoint_left_leg(i) = lower_torque_limit;
            }
        }

        torque_setpoint setpoint;

        setpoint.tau1 = tau_setpoint_left_leg(0);
        setpoint.tau2 = tau_setpoint_left_leg(1);
        setpoint.tau3 = tau_setpoint_left_leg(2);
        setpoint.tau4 = tau_setpoint_left_leg(3);
        setpoint.tau5 = tau_setpoint_left_leg(4);

        //std::cout << "Tau3: " << tau_setpoint_left_leg(2) << std::endl;

        //std::cout << "before publish in left leg" << std::endl;

        stringstream s;

        s << setpoint.tau1 << "|" << setpoint.tau2 << "|" << setpoint.tau3 << "|" << setpoint.tau4 << "|" << setpoint.tau5;

        //zcm_context.publish(left_leg_torque_setpoint_channel, &setpoint);

        sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len);

        //std::cout << "Sent: " << G_left_leg(0) << " , " << G_left_leg(1) << " , " << G_left_leg(2) << " , " << G_left_leg(3) << " , " << G_left_leg(4) << std::endl;

        end = high_resolution_clock::now();

        duration = duration_cast<microseconds>(end - start).count();
        long long remainder = (torque_calculation_interval - duration) * 1e+03;
        deadline.tv_nsec = remainder;
        deadline.tv_sec = 0;
        clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
    }
}

class Handler
{
    public:
        ~Handler() {}

        void handleLeftLegMessage(const zcm::ReceiveBuffer* rbuf,
                           const string& chan,
                           const leg_state *state)
        {
            std::cout << "Got message, updating state..." << std::endl;
            update_q_left_leg(state->theta1, state->theta2, state->theta3, state->theta4, state->theta5);
            update_q_dot_left_leg(state->theta1_dot, state->theta2_dot, state->theta3_dot, state->theta4_dot, state->theta5_dot);
        }
};

int main()
{
    if(!zcm_context.good()) {
        std::cout << "ZCM isn't feeling too well, exiting..." << std::endl; 
        return 1;
    }

    pos_desired_left_leg << 0.2, -0.4, -0.5;
    vel_desired_left_leg << 0, 0, 0;
    accel_desired_left_leg << 0, 0, 0;

    Handler handlerObject;

    auto left_leg_subs = zcm_context.subscribe(left_leg_state_channel, &Handler::handleLeftLegMessage, &handlerObject);
    //zcm_context.subscribe(right_leg_state_channel, &Handler::handleRightLegMessage, &handlerObject);

    zcm_context.start();
    zcm_context.run();
    
    //left_leg_state_thread = std::thread(std::bind(update_left_leg_state));
    //right_leg_state_thread = std::thread(std::bind(update_right_leg_state));
    left_leg_torque_thread = std::thread(std::bind(calculate_left_leg_torques));
    //right_leg_torque_thread = std::thread(std::bind(calculate_right_leg_torques));

    // update_J_foot_dot_left_leg(0,1,2,3,4,5,6,7,8,9);

    // std::cout << J_foot_dot_left_leg << std::endl;

    std::cout << "omega_desired is currently:" << omega_desired << std::endl;

    while(true) {

    }
}

//COMPILE: g++ controller.cpp leg_state.hpp torque_setpoint.hpp -o controller -lzcm -pthread -I .
//Also, ALWAYS run sudo due to IPC rights