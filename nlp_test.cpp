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
#include <Eigen_unsupported/Eigen/MatrixFunctions>


using namespace casadi;

using namespace std;
using namespace std::chrono;

int udp_buffer_size = 4096;
const int udp_port = 4200; // UDP port for communication between gazebosim and controller code
const int mpc_port = 4801;

unsigned long long factorial(long factor) {
    unsigned long long temp = 1;

    for(int i = 1; i <= factor; ++i)
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
    Eigen::Matrix<double, n+m, n+m> A_B = Eigen::ArrayXXd::Zero(n+m, n+m);

    A_B.block<n, n>(0, 0) = A_c_temp;
    A_B.block<n, m>(0, n) = B_c_temp;

    Eigen::MatrixXd e_A_B = (A_B * dt).exp();

    A_d_temp = e_A_B.block<n, n>(0, 0).eval();
    B_d_temp = e_A_B.block<n, m>(0, n).eval();
}

Eigen::Matrix<double, n, 1> step_discrete_model(Eigen::Matrix<double, n, 1> x, Eigen::Matrix<double, m, 1> u, double r_x_left_temp, double r_x_right_temp, double r_y_left_temp, double r_y_right_temp, double r_z_left_temp, double r_z_right_temp) {

    double phi_t_temp = x(0, 0);
    double theta_t_temp = x(1, 0);
    double psi_t_temp = x(2, 0);

    Eigen::Matrix<double, 3, 3> I_world_temp = Eigen::ArrayXXd::Zero(3, 3);

    I_world_temp << (Ixx*cos(psi_t_temp) + Iyx*sin(psi_t_temp))*cos(psi_t_temp) + (Ixy*cos(psi_t_temp) + Iyy*sin(psi_t_temp))*sin(psi_t_temp), -(Ixx*cos(psi_t_temp) + Iyx*sin(psi_t_temp))*sin(psi_t_temp) + (Ixy*cos(psi_t_temp) + Iyy*sin(psi_t_temp))*cos(psi_t_temp), Ixz*cos(psi_t_temp) + Iyz*sin(psi_t_temp), (-Ixx*sin(psi_t_temp) + Iyx*cos(psi_t_temp))*cos(psi_t_temp) + (-Ixy*sin(psi_t_temp) + Iyy*cos(psi_t_temp))*sin(psi_t_temp), -(-Ixx*sin(psi_t_temp) + Iyx*cos(psi_t_temp))*sin(psi_t_temp) + (-Ixy*sin(psi_t_temp) + Iyy*cos(psi_t_temp))*cos(psi_t_temp), -Ixz*sin(psi_t_temp) + Iyz*cos(psi_t_temp), Ixy*sin(psi_t_temp) + Izx*cos(psi_t_temp), Ixy*cos(psi_t_temp) - Izx*sin(psi_t_temp), Izz;

    Eigen::Matrix<double, 3, 3> r_left_skew_symmetric_test = Eigen::ArrayXXd::Zero(3, 3);
    Eigen::Matrix<double, 3, 3> r_right_skew_symmetric_test = Eigen::ArrayXXd::Zero(3, 3);

    r_left_skew_symmetric_test << 0, -r_z_left_temp, r_y_left_temp,
                                r_z_left_temp, 0, -r_x_left_temp,
                                -r_y_left_temp, r_x_left_temp, 0;
            
    r_right_skew_symmetric_test << 0, -r_z_right_temp, r_y_right_temp,
                                r_z_right_temp, 0, -r_x_right_temp,
                                -r_y_right_temp, r_x_right_temp, 0;

    Eigen::Matrix<double, n, n> A_c_temp_temp = Eigen::ArrayXXd::Zero(n, n);
    Eigen::Matrix<double, n, m> B_c_temp_temp = Eigen::ArrayXXd::Zero(n, m);

    A_c_temp_temp << 0, 0, 0, 0, 0, 0, cos(psi_t_temp)*cos(theta_t_temp), sin(phi_t_temp)*sin(theta_t_temp)*cos(psi_t_temp) - sin(psi_t_temp)*cos(phi_t_temp), sin(phi_t_temp)*sin(psi_t_temp) + sin(theta_t_temp)*cos(phi_t_temp)*cos(psi_t_temp), 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, sin(psi_t_temp)*cos(theta_t_temp), sin(phi_t_temp)*sin(psi_t_temp)*sin(theta_t_temp) + cos(phi_t_temp)*cos(psi_t_temp), -sin(phi_t_temp)*cos(psi_t_temp) + sin(psi_t_temp)*sin(theta_t_temp)*cos(phi_t_temp), 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, -sin(theta_t_temp), sin(phi_t_temp)*cos(theta_t_temp), cos(phi_t_temp)*cos(theta_t_temp), 0, 0, 0, 0,
            
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

    B_c_temp_temp << 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,   
                    I_world_temp.inverse() * r_left_skew_symmetric_test, I_world_temp.inverse() * r_right_skew_symmetric_test,
                    1/m_value, 0, 0, 1/m_value, 0, 0,
                    0, 1/m_value, 0, 0, 1/m_value, 0,
                    0, 0, 1/m_value, 0, 0, 1/m_value,
                    0, 0, 0, 0, 0, 0;

    Eigen::Matrix<double, n, n> A_d_temp_temp = Eigen::ArrayXXd::Zero(n, n);
    Eigen::Matrix<double, n, m> B_d_temp_temp = Eigen::ArrayXXd::Zero(n, m);

    discretize_state_space_matrices(A_c_temp_temp, B_c_temp_temp, dt, A_d_temp_temp, B_d_temp_temp);

    return A_d_temp_temp * x + B_d_temp_temp * u;
}

int main() {
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

    opts["print_time"] = 0;
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
    
    for(int k = 0; k < n; ++k) {
        X_t(k*n+0, 0) = x_t(k, 0);
    }
    
    Eigen::Matrix<double, m*N, 1> U_t = Eigen::ArrayXXd::Zero(m*N, 1); // Same here

    Eigen::Matrix<double, n, N> x_ref = Eigen::ArrayXXd::Zero(n, N);

    static const int P_rows = n;
    static const int P_cols = 1 + N + n * N + m * N + N * m;
    Eigen::Matrix<double, P_rows, P_cols> P_param = Eigen::ArrayXXd::Zero(P_rows, P_cols);

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

        std::cout << "-----------------------------------------------------------------------------\nr_left at beginning of iteration: " << r_x_left << "," << r_y_left << "," << r_z_left << ", r_right at beginning of iteration: " << r_x_right << "," << r_y_right << "," << r_z_right << std::endl;

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

            D_vector.block<m, m>(0, k*m) = D_k.eval();
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
            Eigen::Matrix<double, 3, 1> adjusted_pos_vector_left = (Eigen::Matrix<double, 3,1>() << (double)x_t(3, 0) - hip_offset, x_t(4, 0), x_t(5, 0)).finished();
            Eigen::Matrix<double, 3, 1> adjusted_pos_vector_right = (Eigen::Matrix<double, 3,1>() << (double)x_t(3, 0) + hip_offset, x_t(4, 0), x_t(5, 0)).finished();
            
            Eigen::Matrix<double, 3, 1> vel_vector = (Eigen::Matrix<double, 3, 1>() << x_t(9, 0), x_t(10, 0), x_t(11, 0)).finished();
            Eigen::Matrix<double, 3, 1> vel_desired_vector = (Eigen::Matrix<double, 3, 1>() << vel_x_desired, vel_y_desired, vel_z_desired).finished();
            Eigen::Matrix<double, 3, 1> omega_desired_vector = (Eigen::Matrix<double, 3, 1>() << omega_x_desired, omega_y_desired, omega_z_desired).finished();

            left_foot_pos_world = adjusted_pos_vector_left /*+ (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(x_t(5, 0)) / 9.81) * vel_vector.cross(omega_desired_vector)*/;
            right_foot_pos_world = adjusted_pos_vector_right /*+ (t_stance/2) * vel_vector + gait_gain * (vel_vector - vel_desired_vector) + 0.5 * sqrt(abs(x_t(5, 0)) / 9.81) * vel_vector.cross(omega_desired_vector)*/;
            
            if (left_foot_pos_world(0, 0) - (double)x_t(3, 0) > r_x_limit) {
                left_foot_pos_world(0, 0) = (double)x_t(3, 0) + r_x_limit;
            }
            else if (left_foot_pos_world(0, 0) - (double)x_t(3, 0) < -r_x_limit) {
                left_foot_pos_world(0, 0) = (double)x_t(3, 0) - r_x_limit;
            }

            if (right_foot_pos_world(0, 0) - (double)x_t(3, 0) > r_x_limit) {
                right_foot_pos_world(0, 0) = (double)x_t(3, 0) + r_x_limit;
            }
            else if (right_foot_pos_world(0, 0) - (double)x_t(3, 0) < -r_x_limit) {
                right_foot_pos_world(0, 0) = (double)x_t(3, 0) - r_x_limit;
            }
        }

        r_x_left = left_foot_pos_world(0, 0) - (double)x_t(3, 0);
        r_x_right = right_foot_pos_world(0, 0) - (double)x_t(3, 0);

        r_y_left = left_foot_pos_world(1, 0) - (double)x_t(4, 0);
        r_y_right = right_foot_pos_world(1, 0) - (double)x_t(4, 0);

        r_x_left = -hip_offset;
        r_x_right = hip_offset;
        r_y_left = r_y_right = 0;

        r_z_left = -x_t(5, 0);
        r_z_right = -x_t(5, 0);

        std::cout << "r_left after foot pos update: " << r_x_left << "," << r_y_left << "," << r_z_left << ", r_right after foot pos update: " << r_x_right << "," << r_y_right << "," << r_z_right << std::endl;

        double pos_x_desired_temp = pos_x_desired;
        double pos_y_desired_temp = pos_y_desired;
        double pos_z_desired_temp = pos_z_desired;

        double phi_desired_temp = phi_desired;
        double theta_desired_temp = theta_desired;
        double psi_desired_temp = psi_desired;

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

        double r_x_left_prev = r_x_left;
        double r_x_right_prev = r_x_right;

        double r_y_left_prev = r_y_left;
        double r_y_right_prev = r_y_right;

        Eigen::Matrix<double, 3, 1> left_foot_pos_world_prev = left_foot_pos_world;
        Eigen::Matrix<double, 3, 1> right_foot_pos_world_prev = right_foot_pos_world;

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
                phi_t = (double)x_t(0, 0);
                theta_t = (double)x_t(1, 0);
                psi_t = (double)x_t(2, 0);

                vel_x_t = (double)x_t(9, 0);
                vel_y_t = (double)x_t(10, 0);
                vel_z_t = (double)x_t(11, 0);

                pos_x_t = (double)x_t(3, 0);
                pos_y_t = (double)x_t(4, 0);
                pos_z_t = (double)x_t(5, 0);
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
            r_x_right = right_foot_pos_world(0, 0) - pos_x_t;

            r_y_left = left_foot_pos_world(1, 0) - pos_y_t;
            r_y_right = right_foot_pos_world(1, 0) - pos_y_t;

            r_x_left = -hip_offset;
            r_x_right = hip_offset;
            r_y_left = r_y_right = 0;

            r_z_left = -pos_z_t;
            r_z_right = -pos_z_t;

            std::cout << "r_left after discretization update: " << r_x_left << "," << r_y_left << "," << r_z_left << ", r_right after discretization update: " << r_x_right << "," << r_y_right << "," << r_z_right << std::endl;

            I_world << (Ixx*cos(psi_t) + Iyx*sin(psi_t))*cos(psi_t) + (Ixy*cos(psi_t) + Iyy*sin(psi_t))*sin(psi_t), -(Ixx*cos(psi_t) + Iyx*sin(psi_t))*sin(psi_t) + (Ixy*cos(psi_t) + Iyy*sin(psi_t))*cos(psi_t), Ixz*cos(psi_t) + Iyz*sin(psi_t), (-Ixx*sin(psi_t) + Iyx*cos(psi_t))*cos(psi_t) + (-Ixy*sin(psi_t) + Iyy*cos(psi_t))*sin(psi_t), -(-Ixx*sin(psi_t) + Iyx*cos(psi_t))*sin(psi_t) + (-Ixy*sin(psi_t) + Iyy*cos(psi_t))*cos(psi_t), -Ixz*sin(psi_t) + Iyz*cos(psi_t), Ixy*sin(psi_t) + Izx*cos(psi_t), Ixy*cos(psi_t) - Izx*sin(psi_t), Izz;

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

            P_param.block<n, n>(0, 1 + N + (i*n)) = A_d_t.eval();
            P_param.block<n, m>(0, 1 + N + n * N + (i*m)) = B_d_t.eval();
        }

        left_foot_pos_world = left_foot_pos_world_prev;
        right_foot_pos_world = right_foot_pos_world_prev;

        r_x_left = r_x_left_prev;
        r_x_right = r_x_right_prev;

        r_y_left = r_y_left_prev;
        r_y_right = r_y_right_prev;

        r_z_left = r_z_right = -x_t(5, 0);

        std::cout << "r_left after discretization loop: " << r_x_left << "," << r_y_left << "," << r_z_left << ", r_right after discretization loop: " << r_x_right << "," << r_y_right << "," << r_z_right << std::endl;

        size_t rows_P_param = P_param.rows();
        size_t cols_P_param = P_param.cols();

        //std::cout << "P_param:\n" << P_param << std::endl;

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

        std::cout << "r_left after stepping model: " << r_x_left << "," << r_y_left << "," << r_z_left << ", r_right after stepping model: " << r_x_right << "," << r_y_right << "," << r_z_right << std::endl;

        stringstream s;

        // s << u_t(0) << "|" << u_t(1) << "|" << u_t(2) << "|" << u_t(3) << "|" << u_t(4) << "|" << u_t(5) << "|" << r_x_left << "|" << r_y_left << "|" << r_z_left << "|" << r_x_right << "|" << r_y_right << "|" << r_z_right; // Write torque setpoints to stringstream
        // sendto(sockfd, (const char *)s.str().c_str(), strlen(s.str().c_str()), MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len);
        
        ofstream data_file;
        data_file.open(".././plot_data/mpc_log.csv", ios::app); // Open csv file in append mode
        data_file << total_iterations * dt << "," << x_t(0, 0) << "," << x_t(1, 0) << "," << x_t(2, 0) << "," << x_t(3, 0) << "," << x_t(4, 0) << "," << x_t(5, 0) << "," << x_t(6, 0) << "," << x_t(7, 0) << "," << x_t(8, 0) << "," << x_t(9, 0) << "," << x_t(10, 0) << "," << x_t(11, 0) << "," << x_t(12, 0)
                << "," << u_t(0) << "," << u_t(1) << "," << u_t(2) << "," << u_t(3) << "," << u_t(4) << "," << u_t(5) 
                << "," << r_x_left << "," << r_y_left << "," << r_z_left << "," << r_x_right << "," << r_y_right << "," << r_z_right << ",0" << std::endl; // Zero at the end has to be replace with predicted delay compensation state!
        data_file.close(); // Close csv file again. This way thread abort should (almost) never leave file open.

        X_t.block<n*N, 1>(0, 0) = solution_variables.block<n*N, 1>(n, 0).eval();
        X_t.block<n, 1>(n*N, 0) = solution_variables.block<n, 1>(n*N, 0).eval();

        U_t.block<m*(N-1), 1>(0, 0) = solution_variables.block<m*(N-1), 1>(n*(N+1) + m, 0).eval();
        U_t.block<m, 1>(m*(N-1), 0) = solution_variables.block<m, 1>(n*(N+1)+m*(N-1), 0).eval();

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

    return 0;
}