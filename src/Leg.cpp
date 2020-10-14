#include "include/Leg.hpp"

Leg::Leg(double hip_offset_x, double hip_offset_y, double hip_offset_z) {
    // Initiate damping ratio matrix, desired natural frequency, orientation gains as well as desired trajectory to avoid null pointer
    h << 0.6, 0, 0,
        0, 0.6, 0,
        0, 0, 0.6;
    
    omega_desired << 8.0 * M_PI, 16.0 * M_PI, 8.0 * M_PI;

    pos_desired << 0, 0, -1.115, 0, 0; // Cartesian xyz + euler roll and pitch
    vel_desired << 0, 0, 0, 0, 0; // Cartesian xyz + euler roll and pitch
    accel_desired << 0, 0, 0; // Cartesian xyz

    Kp_orientation = 9;
    Kd_orientation = 0.15;
}

long long iteration_counter = 0;

double hip_offset_x;
double hip_offset_y;
double hip_offset_z;

double theta1;
double theta2;
double theta3;
double theta4;
double theta5;

double theta1dot;
double theta2dot;
double theta3dot;
double theta4dot;
double theta5dot;

double trajectory_start_time;
double t_stance_remainder;

// Boolean representing foot state. True means foot is in the air, i.e. no contact, false means foot is in stance phase. i.e. contact
bool swing_phase;

std::mutex q_mutex, q_dot_mutex, foot_pos_world_mutex, foot_pos_world_desired_mutex, lift_off_pos_mutex, lift_off_vel_mutex, t_stance_remainder_mutex, foot_pos_body_frame_mutex,
                    trajectory_start_time_mutex, foot_trajectory_mutex, foot_pos_desired_world_mutex;

Eigen::Matrix<double, 5, 1> q; // Leg angle vector / Model state
Eigen::Matrix<double, 5, 1> q_dot; // Leg angular velocity vector / Differentiated model state

Eigen::Matrix<double, 5, 1> C; // matrix containing the result of C * q_dot and the other terms based on the jupyter notebook
Eigen::Matrix<double, 5, 5> B; // mass and inertia matrix of the leg model
Eigen::Matrix<double, 5, 1> G; // gravity vector of the leg model. If directly applied as torques to each joint, it should compensate for gravity.

Eigen::Matrix<double, 3, 5> J_foot; // Jacobian of the foot / end effector, also called the contact Jacobian.
Eigen::Matrix<double, 5, 5> J_foot_combined; // Combined Jacobian with geometric positional part and analytical orientation part.
Eigen::Matrix<double, 3, 5> J_foot_dot; // Time derivative of the contact / end effector Jacobian,

Eigen::Matrix<double, 3, 3> Lambda; // "Desired Inertia matrix" of the leg, based on Jacobian and inertia matrix B / M

Eigen::Matrix<double, 5, 5> Kp; // Cartesian Position gain matrix for calculation of torque setpoint
Eigen::Matrix<double, 5, 5> Kd; // Derivative / Cartesian Velocity for calculation of torque setpoint

Eigen::Matrix<double, 5, 1> tau_ff; // Vector containing feedforward torque based on Coriolis, Centrifugal, gravity and feed-forward acceleration terms.
Eigen::Matrix<double, 5, 1> tau_setpoint; // Final torque setpoint calculated from above matrices and feedforward torque added.

Eigen::Matrix<double, 5, 1> foot_pos; // Cartesian foot / end-effector position, hip frame
Eigen::Matrix<double, 3, 1> foot_pos_body_frame; // Body frame
Eigen::Matrix<double, 3, 1> foot_pos_desired_body_frame; // Body frame
Eigen::Matrix<double, 3, 1> foot_pos_desired_world; // World frame
Eigen::Matrix<double, 3, 1> foot_pos_world; // foot position in world frame
Eigen::Matrix<double, 3, 1> lift_off_pos; // Body frame
Eigen::Matrix<double, 3, 1> foot_pos_world_discretization; // World frame

Eigen::Matrix<double, 5, 1> foot_vel; // Cartesian foot/ end-effector velocity
Eigen::Matrix<double, 3, 1> lift_off_vel; // Body frame

Eigen::Matrix<double, 5, 1> pos_desired; // Desired cartesian foot / end-effector position + orientation (roll and yaw)
Eigen::Matrix<double, 5, 1> vel_desired; // Desired cartesian foot / end-effector velocity + angular velocity
Eigen::Matrix<double, 3, 1> accel_desired; // Desired cartesian foot / end-effector acceleration

Eigen::Matrix<double, 3, 3> h; // Damping ratio matrix
Eigen::Matrix<double, 3, 1> omega_desired; // Desired natural frequency of the leg

Eigen::Matrix<double, 334, 6> foot_trajectory;


double Kp_orientation;
double Kd_orientation;

// Euler Angle definitions:
// roll - around x - alpha - phi
// pitch - around y - beta - theta
// yaw - around z - gamma - psi

double phi = 0; // Roll, rotation around X, alpha
double theta = 0; // Pitch, rotation around Y, beta
double psi = 0; // Yaw, rotation arond Z, gamma

static leg_config config;

void Leg::update_torque_setpoint() {
    update_tau_setpoint(J_foot_combined, Kp, pos_desired, foot_pos, Kd, vel_desired, foot_vel, tau_ff, tau_setpoint);

    for(int i = 0; i < 5; ++i) { // Loop through each torque setpoint vector element
            constrain(tau_setpoint(i), config.lower_torque_limit, config.upper_torque_limit); // constrain element based on global torque limits
        }
        constrain(tau_setpoint(4), -5, 5);
}

void Leg::update_foot_pos_body_frame(Eigen::Matrix<double, 13, 1> &com_state) {

    double phi_com = com_state(0, 0);
    double theta_com = com_state(1, 0);
    double psi_com = com_state(2, 0);

    double pos_x_com = com_state(3, 0);
    double pos_y_com = com_state(4, 0);
    double pos_z_com = com_state(5, 0);

    double vel_x_com = com_state(9, 0);
    double vel_y_com = com_state(10, 0);
    double vel_z_com = com_state(11, 0);

    Eigen::Matrix<double, 4, 4> H_world_body = (Eigen::Matrix<double, 4, 4>() << cos(psi_com)*cos(theta_com), sin(psi_com)*cos(theta_com), -sin(theta_com), -pos_x_com*cos(psi_com)*cos(theta_com) - pos_y_com*sin(psi_com)*cos(theta_com) + pos_z_com*sin(theta_com), 
                                            sin(phi_com)*sin(theta_com)*cos(psi_com) - sin(psi_com)*cos(phi_com), sin(phi_com)*sin(psi_com)*sin(theta_com) + cos(phi_com)*cos(psi_com), sin(phi_com)*cos(theta_com), -pos_x_com*sin(phi_com)*sin(theta_com)*cos(psi_com) + pos_x_com*sin(psi_com)*cos(phi_com) - pos_y_com*sin(phi_com)*sin(psi_com)*sin(theta_com) - pos_y_com*cos(phi_com)*cos(psi_com) - pos_z_com*sin(phi_com)*cos(theta_com), 
                                            sin(phi_com)*sin(psi_com) + sin(theta_com)*cos(phi_com)*cos(psi_com), -sin(phi_com)*cos(psi_com) + sin(psi_com)*sin(theta_com)*cos(phi_com), cos(phi_com)*cos(theta_com), -pos_x_com*sin(phi_com)*sin(psi_com) - pos_x_com*sin(theta_com)*cos(phi_com)*cos(psi_com) + pos_y_com*sin(phi_com)*cos(psi_com) - pos_y_com*sin(psi_com)*sin(theta_com)*cos(phi_com) - pos_z_com*cos(phi_com)*cos(theta_com), 
                                            0, 0, 0, 1).finished();

    // Convert from hip to body frame
    Eigen::Matrix<double, 4, 4> H_hip_body = (Eigen::Matrix<double, 4, 4>() << 1, 0, 0, Leg::hip_offset_x,
                                                                                0, 1, 0, 0,
                                                                                0, 0, 1, Leg::hip_offset_z, // Torso Z - Hip Z in Gazebo SDF
                                                                                0, 0, 0, 1).finished();

    Leg::foot_pos_body_frame_mutex.lock();
    Leg::foot_pos_body_frame = (H_hip_body * (Eigen::Matrix<double, 4, 1>() << foot_pos.block<3,1>(0, 0), 1).finished()).block<3,1>(0, 0);
    Leg::foot_pos_body_frame_mutex.unlock();

    stringstream temp;
    temp << foot_pos_body_frame;
    print_threadsafe(temp.str(), "foot_pos_body_frame in update function", INFO);

    temp.str(std::string());
    temp << (H_world_body.inverse() * (Eigen::Matrix<double, 4, 1>() << foot_pos_body_frame.block<3,1>(0, 0), 1).finished()).block<3,1>(0, 0);
    print_threadsafe(temp.str(), "foot_pos_left_world in update function", INFO);
}

void Leg::update_foot_trajectory(Eigen::Matrix<double, 13, 1> &com_state, Eigen::Matrix<double, 3, 1> next_body_vel, double t_stance, double time) {

    double phi_com = com_state(0, 0);
    double theta_com = com_state(1, 0);
    double psi_com = com_state(2, 0);

    double pos_x_com = com_state(3, 0);
    double pos_y_com = com_state(4, 0);
    double pos_z_com = com_state(5, 0);

    double vel_x_com = com_state(9, 0);
    double vel_y_com = com_state(10, 0);
    double vel_z_com = com_state(11, 0);

    Eigen::Matrix<double, 4, 4> H_world_body = (Eigen::Matrix<double, 4, 4>() << cos(psi_com)*cos(theta_com), sin(psi_com)*cos(theta_com), -sin(theta_com), -pos_x_com*cos(psi_com)*cos(theta_com) - pos_y_com*sin(psi_com)*cos(theta_com) + pos_z_com*sin(theta_com), 
                                            sin(phi_com)*sin(theta_com)*cos(psi_com) - sin(psi_com)*cos(phi_com), sin(phi_com)*sin(psi_com)*sin(theta_com) + cos(phi_com)*cos(psi_com), sin(phi_com)*cos(theta_com), -pos_x_com*sin(phi_com)*sin(theta_com)*cos(psi_com) + pos_x_com*sin(psi_com)*cos(phi_com) - pos_y_com*sin(phi_com)*sin(psi_com)*sin(theta_com) - pos_y_com*cos(phi_com)*cos(psi_com) - pos_z_com*sin(phi_com)*cos(theta_com), 
                                            sin(phi_com)*sin(psi_com) + sin(theta_com)*cos(phi_com)*cos(psi_com), -sin(phi_com)*cos(psi_com) + sin(psi_com)*sin(theta_com)*cos(phi_com), cos(phi_com)*cos(theta_com), -pos_x_com*sin(phi_com)*sin(psi_com) - pos_x_com*sin(theta_com)*cos(phi_com)*cos(psi_com) + pos_y_com*sin(phi_com)*cos(psi_com) - pos_y_com*sin(psi_com)*sin(theta_com)*cos(phi_com) - pos_z_com*cos(phi_com)*cos(theta_com), 
                                            0, 0, 0, 1).finished();
    
    foot_pos_world_desired_mutex.lock();
    Eigen::Matrix<double, 3, 1> pos_desired_left_leg_body_frame = (H_world_body * (Eigen::Matrix<double, 4, 1>() << foot_pos_desired_world, 1).finished()).block<3, 1>(0, 0);
    foot_pos_world_desired_mutex.unlock();
    
    update_foot_pos_body_frame(com_state);
    
    double step_height_world = 0.15;
    double step_height_body = (H_world_body * (Eigen::Matrix<double, 4, 1>() << 0, 0, step_height_world, 1).finished())(2, 0);

    lift_off_pos_mutex.lock();
    lift_off_vel_mutex.lock();
    foot_trajectory_mutex.lock();
    
    foot_trajectory = get_swing_trajectory(lift_off_pos,
        (Eigen::Matrix<double, 3, 1>() << (lift_off_pos(0, 0) - foot_pos_desired_body_frame(0, 0)) / 2, (lift_off_pos(1, 0) - foot_pos_desired_body_frame(1, 0)) / 2, step_height_body).finished(), foot_pos_desired_body_frame, 
        lift_off_vel, -next_body_vel,
        t_stance);
    
    trajectory_start_time_mutex.lock();
    trajectory_start_time = time;
    trajectory_start_time_mutex.unlock();
    
    std::stringstream temp;
    temp << "lift_off_pos: " << lift_off_pos(0, 0) << "," << lift_off_pos(1, 0) << "," << lift_off_pos(2, 0)
                << "\nleft_foot_pos_world_desired: " << foot_pos_world(0, 0) << "," << foot_pos_world(1, 0) << "," << foot_pos_world(2, 0)
                << "\ntarget_foot_pos_body: " << foot_pos_desired_body_frame(0, 0) << "," << foot_pos_desired_body_frame(1, 0) << "," << foot_pos_desired_body_frame(2, 0) 
                << "\ntarget_foot_pos_world: " << foot_pos_desired_world(0, 0) << "," << foot_pos_desired_world(1, 0) << "," << foot_pos_desired_world(2, 0);
    print_threadsafe(temp.str(), "mpc_thread", INFO);
    
    foot_trajectory_mutex.unlock();
    lift_off_pos_mutex.unlock();
    lift_off_vel_mutex.unlock();
}


void Leg::update() {
    q_mutex.lock();
    update_q(theta1, theta2, theta3, theta4, theta5, q, config);
    q_mutex.unlock();

    q_dot_mutex.lock();
    update_q_dot(theta1dot, theta2dot, theta3dot, theta4dot, theta5dot, q_dot, config);
    q_dot_mutex.unlock();


    update_orientation(theta1, theta2, theta3, theta4, theta5, phi, theta, psi);

    update_B(theta1, theta2, theta3, theta4, theta5, theta1dot, theta2dot, theta3dot, theta4dot, theta5dot, B, config);

    update_J_foot(theta1, theta2, theta3, theta4, theta5, J_foot, config);

    update_J_foot_combined(theta1, theta2, theta3, theta4, theta5, theta1dot, theta2dot, theta3dot, theta4dot, theta5dot, J_foot_combined, config);

    update_J_foot_dot(theta1, theta2, theta3, theta4, theta5, theta1dot, theta2dot, theta3dot, theta4dot, theta5dot, J_foot_dot, config);

    //Set singular Jacobians to zero, filter and constrain elements

    for(int i = 0, nCols = J_foot.cols(), nRows = J_foot.rows(); i < nCols; ++i) {
        for(int j = 0; i < nRows; ++i) {
            if(isnan(J_foot(j, i)) || isinf(J_foot(j, i))) {
                J_foot(j, i) = 0;
            }
        }
    }

    for(int i = 0, nCols = J_foot_dot.cols(), nRows = J_foot_dot.rows(); i < nCols; ++i) {
        for(int j = 0; i < nRows; ++i) {
            if(isnan(J_foot_dot(j, i)) || isinf(J_foot_dot(j, i))) {
                J_foot_dot(j, i) = 0;
            }
        }
    }

    for(int i = 0, nCols = J_foot_combined.cols(), nRows = J_foot_combined.rows(); i < nCols; ++i) {
        for(int j = 0; i < nRows; ++i) {
            if(isnan(J_foot_combined(j, i)) || isinf(J_foot_combined(j, i))) {
                J_foot_combined(j, i) = 0;
            }
        }
    }

    update_G(theta1, theta2, theta3, theta4, theta5, G, config);

    update_C(theta1, theta2, theta3, theta4, theta5, theta1dot, theta2dot, theta3dot, theta4dot, theta5dot, C, config);

    update_Lambda(J_foot, B, Lambda);
    
    for(int i = 0, nCols = Lambda.cols(), nRows = Lambda.rows(); i < nCols; ++i) {
        for(int j = 0; i < nRows; ++i) {
            if(isnan(Lambda(j, i)) || isinf(Lambda(j, i))) {
                Lambda(j, i) = 0;
            }
        }
    }

    q_dot_mutex.lock();
    update_tau_ff(q_dot, C, J_foot, J_foot_dot, Lambda, accel_desired, q_dot, tau_ff);
    q_dot_mutex.unlock();

    update_Kp(Lambda, omega_desired, Kp_orientation, Kp);
    update_Kd(Lambda, omega_desired, h, Kd_orientation, Kd);
    
    update_foot_pos(theta1, theta2, theta3, theta4, theta5, phi, psi, foot_pos, config);
    
    Leg::q_dot_mutex.lock();
    update_foot_vel(J_foot_combined, Leg::q_dot, foot_vel);
    Leg::q_dot_mutex.unlock();

    for(int i = 0; i < 5; ++i) { // Loop through each torque setpoint vector element
        constrain(tau_setpoint(i), Leg::config.lower_torque_limit, Leg::config.upper_torque_limit); // constrain element based on global torque limits
    }
    constrain(tau_setpoint(4), -5, 5);

    Leg::iteration_counter++;
}