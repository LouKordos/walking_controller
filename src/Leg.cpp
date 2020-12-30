#include "include/Leg.hpp"

Leg::Leg(const double hip_offset_x, const double hip_offset_y, const double hip_offset_z, const int contact_state_port) : contactState(contact_state_port) {
    // Initiate damping ratio matrix, desired natural frequency, orientation gains as well as desired trajectory to avoid null pointer
    h << 0.6, 0, 0,
        0, 0.6, 0,
        0, 0, 0.6;
    
    omega_desired << 10.0 * M_PI, 16.0 * M_PI, 10.0 * M_PI;

    pos_desired << 0, 0, -1.115, 0, 0; // Cartesian xyz + euler roll and pitch
    vel_desired << 0, 0, 0, 0, 0; // Cartesian xyz + euler roll and pitch
    accel_desired << 0, 0, 0; // Cartesian xyz

    update_foot_pos(theta1, theta2, theta3, theta4, theta5, phi, psi, foot_pos, config);

    Kp_orientation = 9;
    Kd_orientation = 0.15;

    this->hip_offset_x = hip_offset_x;
    this->hip_offset_y = hip_offset_y;
    this->hip_offset_z = hip_offset_z;

    // Convert from hip to body frame, look at Leg Class Instantiation for explanation
    H_hip_body = (Eigen::Matrix<double, 4, 4>() << 1, 0, 0, hip_offset_x,
                                                    0, 1, 0, hip_offset_y,
                                                    0, 0, 1, hip_offset_z, // Torso Z - Hip Z in Gazebo SDF
                                                    0, 0, 0, 1).finished();

    foot_trajectory = CartesianTrajectory();
}

long long iteration_counter = 0;

// Euler Angle definitions:
// roll - around x - alpha - phi
// pitch - around y - beta - theta
// yaw - around z - gamma - psi

double phi = 0; // Roll, rotation around X, alpha
double theta = 0; // Pitch, rotation around Y, beta
double psi = 0; // Yaw, rotation arond Z, gamma

static leg_config config;

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

    Leg::foot_pos_body_frame_mutex.lock();
    Leg::foot_pos_body_frame = (H_hip_body * (Eigen::Matrix<double, 4, 1>() << foot_pos.block<3,1>(0, 0), 1).finished()).block<3,1>(0, 0);
    Leg::foot_pos_body_frame_mutex.unlock();

    // stringstream temp;
    // temp << "foot_pos_body_frame: " << foot_pos_body_frame;
    // print_threadsafe(temp.str(), "Leg::update_foot_pos_body_frame", INFO);

    // temp.str(std::string());
    // temp << (H_world_body.inverse() * (Eigen::Matrix<double, 4, 1>() << foot_pos_body_frame.block<3,1>(0, 0), 1).finished()).block<3,1>(0, 0);
    // print_threadsafe(temp.str(), "foot_pos_world_frame in update function", INFO);
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
    foot_pos_desired_body_frame_mutex.lock();
    foot_pos_desired_body_frame = (H_world_body * (Eigen::Matrix<double, 4, 1>() << next_foot_pos_world_desired, 1).finished()).block<3, 1>(0, 0);
    foot_pos_desired_body_frame_mutex.unlock();
    foot_pos_world_desired_mutex.unlock();
    
    update_foot_pos_body_frame(com_state);
    
    double step_height_world = 0.15 /*+ 0.4*/;
    double step_height_body = (H_world_body * (Eigen::Matrix<double, 4, 1>() << 0, 0, step_height_world, 1).finished())(2, 0);
    
    lift_off_pos_mutex.lock();
    lift_off_vel_mutex.lock();
    foot_trajectory_mutex.lock();
    
    foot_pos_desired_body_frame_mutex.lock();

    // Eigen::Matrix<double, 3, 1> middle_pos = (Eigen::Matrix<double, 3, 1>() << (lift_off_pos(0, 0) - foot_pos_desired_body_frame(0, 0)) / 2, (lift_off_pos(1, 0) - foot_pos_desired_body_frame(1, 0)) / 2, step_height_body).finished();
    Eigen::Matrix<double, 3, 1> middle_pos = (lift_off_pos + foot_pos_desired_body_frame) / 2;
    middle_pos(2, 0) = step_height_body;

    foot_trajectory.update(lift_off_pos, middle_pos, foot_pos_desired_body_frame, -lift_off_vel, -next_body_vel, t_stance);

    // TEMPORARY: Use 0 velocities for current and target for debugging, use above version for real testing when walking etc.
        // foot_trajectory = get_swing_trajectory(lift_off_pos,
        //     (Eigen::Matrix<double, 3, 1>() << (lift_off_pos(0, 0) - foot_pos_desired_body_frame(0, 0)) / 2, (lift_off_pos(1, 0) - foot_pos_desired_body_frame(1, 0)) / 2, step_height_body).finished(), foot_pos_desired_body_frame, 
        //     lift_off_vel, -next_body_vel,
        //     t_stance);
    
    std::stringstream temp;
    // temp << "lift_off_pos: " << lift_off_pos(0, 0) << "," << lift_off_pos(1, 0) << "," << lift_off_pos(2, 0)
    // //             << "\nnext_body_vel: " << next_body_vel(0, 0) << "," << next_body_vel(1, 0) << "," << next_body_vel(2, 0)
    //                << "\nnext_foot_pos_world_desired: " << next_foot_pos_world_desired(0, 0) << "," << next_foot_pos_world_desired(1, 0) << "," << next_foot_pos_world_desired(2, 0);
    //             // << "\nfoot_pos_world: " << foot_pos_world_desired(0, 0) << "," << foot_pos_world_desired(1, 0) << "," << foot_pos_world_desired(2, 0)
    //             // << "\ntarget_foot_pos_body: " << foot_pos_desired_body_frame(0, 0) << "," << foot_pos_desired_body_frame(1, 0) << "," << foot_pos_desired_body_frame(2, 0) 

    // temp << "Arguments given to trajectory planner (all in body frame):\n" << "initial_pos: " << lift_off_pos(0, 0) << "," << lift_off_pos(1, 0) << "," << lift_off_pos(2, 0)
    //     << "\nmiddle_pos: " << middle_pos(0, 0) << "," << middle_pos(1, 0) << "," << middle_pos(2, 0)
    //     << "\ntarget_pos: " << foot_pos_desired_body_frame(0, 0) << "," << foot_pos_desired_body_frame(1, 0) << "," << foot_pos_desired_body_frame(2, 0)
    //     << "\ninitial_vel: " << -lift_off_vel(0, 0) << "," << -lift_off_vel(1, 0) << "," << -lift_off_vel(2, 0)
    //     << "\ntarget_vel: " << -next_body_vel(0, 0) << "," << -next_body_vel(1, 0) << "," << -next_body_vel(2, 0);
    // print_threadsafe(temp.str(), "mpc_thread", INFO);

    foot_pos_desired_body_frame_mutex.unlock();
    foot_trajectory_mutex.unlock();
    lift_off_pos_mutex.unlock();
    lift_off_vel_mutex.unlock();
}

void Leg::update() {
    q_mutex.lock();
    update_q(theta1, theta2, theta3, theta4, theta5, q);
    q_mutex.unlock();

    q_dot_mutex.lock();
    update_q_dot(theta1dot, theta2dot, theta3dot, theta4dot, theta5dot, q_dot);
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
    update_tau_ff(G, C, J_foot, J_foot_dot, Lambda, accel_desired, q_dot, tau_ff);
    q_dot_mutex.unlock();

    update_Kp(Lambda, omega_desired, Kp_orientation, Kp);
    update_Kd(Lambda, omega_desired, h, Kd_orientation, Kd);
    
    update_foot_pos(theta1, theta2, theta3, theta4, theta5, phi, psi, foot_pos, config);
    
    Leg::q_dot_mutex.lock();
    update_foot_vel(J_foot_combined, Leg::q_dot, foot_vel);
    Leg::q_dot_mutex.unlock();

    Leg::iteration_counter++;
}

void Leg::update_torque_setpoint() {
    update_tau_setpoint(J_foot_combined, Kp, pos_desired, foot_pos, Kd, vel_desired, foot_vel, tau_ff, tau_setpoint);

    for(int i = 0; i < 5; ++i) { // Loop through each torque setpoint vector element
            constrain(tau_setpoint(i, 0), config.lower_torque_limit, config.upper_torque_limit); // constrain element based on global torque limits
    }
    constrain(tau_setpoint(4, 0), -5, 5);
}
