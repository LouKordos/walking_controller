import numpy as np
import matplotlib.pyplot as plt
import os
import numpy as np
import pandas as pd
import math
import argparse
from more_itertools import numeric_range # Prediction plots

parser = argparse.ArgumentParser(description='Generate plots based on csv data.')
parser.add_argument("--show-plots", help="Also show plots using plt.show after generating and saving them to file.", action="store_true")
args = parser.parse_args()

print("Starting plot generation...")

home_dir = os.environ['HOME']

filenames = os.listdir(home_dir + "/dev/walking_controller/plot_data/")

largest_index = 0

for name in filenames:
    try:
        index = int(name.split('.')[0].replace('_left', '').replace('_right', ''))
        if index > largest_index:
            largest_index = index
    except:
        pass
		# print("Invalid parse with filename:", name)

filename = "../plot_data/" + str(largest_index) + "_mpc_log.csv"
print("filename:", filename)

os.mkdir(home_dir + "/Pictures/matplotlib_plots/mpc_log/{}".format(largest_index))

plot_image_dir = home_dir + "/Pictures/matplotlib_plots/mpc_log/{}/".format(largest_index)

dataframe = pd.read_csv(filename)

scaling_factor = 0.75
save_dpi = 500 * scaling_factor

relative_width_pos = 5 # inches I think
relative_height_pos = 100 / 25.4 # inches I think

relative_width_angle = 8 # inches I think
relative_height_angle = 100 / 25.4 # inches I think

relative_width_force = 10 # inches I think
relative_height_force = 100 / 25.4 # inches I think

relative_width_delay = 10 # inches I think
relative_height_delay = 12 # inches I think

angle_fig_size = (relative_width_angle * (1/scaling_factor), relative_height_angle * (1/scaling_factor))
pos_fig_size = (relative_width_pos * (1/scaling_factor), relative_height_pos * (1/scaling_factor))
force_fig_size = (relative_width_force * (1/scaling_factor), relative_height_force * (1/scaling_factor))
delay_fig_size = (relative_width_delay * (1/scaling_factor), relative_height_delay * (1/scaling_factor))

angle_fig = plt.figure(figsize=angle_fig_size, dpi=save_dpi)
angle_ax = angle_fig.add_subplot(111)

linewidth = 0.8

plot_every_predicted = 1  # Only plot in steps of plot_every_predicted, so that plot does not get too crowded

delay_flag = True # Determine whether or not delay compensation is applied
ylim_flag = False # Determines whether or not y limits are added to the plots
show_plots = args.show_plots # Determines whether or not plots should be shown interactively after having saved them
generate_prediction_plots = True

# Determine params based on logs (logs should eventually include those values)
N = len(dataframe['X_t'][0].split(";")) - 1
n = len(dataframe['X_t'][0].split(";")[0].split("|"))
dt = 1.0/50.0
print("N:", N)
print("dt:", dt)

print("Max prev_file_write_time:", max(dataframe["previous_file_write_time"]))

for i in range(int(len(dataframe["t_sim"]))):
    
    if dataframe["previous_logging_time"][i] > 500e+3: # nanoseconds
        print("Spike in logging time detected at t=", dataframe["t_sim"][i], ", logging duration=", dataframe["previous_logging_time"][i])
    
    if dataframe["previous_file_write_time"][i] > 500e+3: # nanoseconds
        print("Spike in log entry file write detected at t=", dataframe["t_sim"][i], ", file write duration=", dataframe["previous_file_write_time"][i])

print("Parameters initialized.")

if delay_flag:
    print("Generating actual vs delay compensated state plots...")

    # Euler angles
    delay_angle_fig, delay_angle_axes = plt.subplots(3, figsize=delay_fig_size, dpi=save_dpi)
    
    delay_angle_axes[0].plot(dataframe["t_sim"], dataframe["phi"], label=r"$\phi$", color=[1.0, 0.0, 1.0], linewidth=linewidth)
    delay_angle_axes[1].plot(dataframe["t_sim"], dataframe["theta"], label=r"$\theta$", color=[0.5, 0.0, 1.0], linewidth=linewidth)
    delay_angle_axes[2].plot(dataframe["t_sim"], dataframe["psi"], label=r"$\psi$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

    delay_angle_axes[0].plot(dataframe["t_sim"][1:], [float(x[0]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$\phi$-compensated (shifted)", linewidth=linewidth)
    delay_angle_axes[1].plot(dataframe["t_sim"][1:], [float(x[1]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$\theta$-compensated (shifted)", linewidth=linewidth)
    delay_angle_axes[2].plot(dataframe["t_sim"][1:], [float(x[2]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$\psi$-compensated (shifted)", linewidth=linewidth)

    delay_angle_axes[0].set_title("Actual Roll vs Delay compensation Roll")
    delay_angle_axes[0].set_ylabel("Phi / Roll [rad]", fontsize=14)

    delay_angle_axes[1].set_title("Actual Pitch vs Delay compensation Pitch")
    delay_angle_axes[1].set_ylabel("Theta / Pitch [rad]", fontsize=14)

    delay_angle_axes[2].set_title("Actual Yaw vs Delay compensation Yaw")
    delay_angle_axes[2].set_ylabel("Psi / Yaw [rad]", fontsize=14)
   
    for index in range(len(delay_angle_axes)):
        delay_angle_axes[index].set_xlabel("Time [s]", fontsize=14)
        delay_angle_axes[index].legend(loc='upper right')
        delay_angle_axes[index].grid()
        #delay_angle_axes[0].set_xlim([0, 10])
        if ylim_flag:
            delay_angle_axes[index].set_ylim([-0.3, 0.3])
    
    # Cartesian position 
    delay_pos_fig, delay_pos_axes = plt.subplots(3, figsize=delay_fig_size, dpi=save_dpi)
    
    delay_pos_axes[0].plot(dataframe["t_sim"], dataframe["pos_x"], label=r"$p_x$", color=[1.0, 0.0, 1.0], linewidth=linewidth)
    delay_pos_axes[1].plot(dataframe["t_sim"], dataframe["pos_y"], label=r"$p_y$", color=[0.5, 0.0, 1.0], linewidth=linewidth)
    delay_pos_axes[2].plot(dataframe["t_sim"], dataframe["pos_z"], label=r"$p_z$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

    delay_pos_axes[0].plot(dataframe["t_sim"][1:], [float(x[3]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$p_x$-compensated (shifted)", linewidth=linewidth)
    delay_pos_axes[1].plot(dataframe["t_sim"][1:], [float(x[4]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$p_y$-compensated (shifted)", linewidth=linewidth)
    delay_pos_axes[2].plot(dataframe["t_sim"][1:], [float(x[5]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$p_z$-compensated (shifted)", linewidth=linewidth)

    delay_pos_axes[0].set_title("Actual X Position vs Delay compensation X Position")
    delay_pos_axes[0].set_ylabel("Pos X [m]", fontsize=14)

    delay_pos_axes[1].set_title("Actual Y Position vs Delay compensation Y Position")
    delay_pos_axes[1].set_ylabel("Pos Y [m]", fontsize=14)

    delay_pos_axes[2].set_title("Actual Z Position vs Delay compensation Z Position")
    delay_pos_axes[2].set_ylabel("Pos Z [m]", fontsize=14)
   
    #delay_pos_axes[0].set_xlim([0, 10])
   
    for index in range(len(delay_pos_axes)):
        delay_pos_axes[index].set_xlabel("Time [s]", fontsize=14)
        delay_pos_axes[index].legend(loc='upper right')
        delay_pos_axes[index].grid()
        #delay_pos_axes[index].set_ylim([-0.3, 0.3])

    # Angular velocities
    delay_angular_vel_fig, delay_angular_vel_axes = plt.subplots(3, figsize=delay_fig_size, dpi=save_dpi)
    
    delay_angular_vel_axes[0].plot(dataframe["t_sim"], dataframe["omega_x"], label=r"$\omega_x$", color=[1.0, 0.0, 1.0], linewidth=linewidth)
    delay_angular_vel_axes[1].plot(dataframe["t_sim"], dataframe["omega_y"], label=r"$\omega_y$", color=[0.5, 0.0, 1.0], linewidth=linewidth)
    delay_angular_vel_axes[2].plot(dataframe["t_sim"], dataframe["omega_z"], label=r"$\omega_z$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

    delay_angular_vel_axes[0].plot(dataframe["t_sim"][1:], [float(x[6]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$\omega_x$-compensated (shifted)", linewidth=linewidth)
    delay_angular_vel_axes[1].plot(dataframe["t_sim"][1:], [float(x[7]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$\omega_y$-compensated (shifted)", linewidth=linewidth)
    delay_angular_vel_axes[2].plot(dataframe["t_sim"][1:], [float(x[8]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$\omega_z$-compensated (shifted)", linewidth=linewidth)

    delay_angular_vel_axes[0].set_title("Actual X Angular Velocity vs Delay compensation X Angular Velocity")
    delay_angular_vel_axes[0].set_ylabel("Omega X [rad/s]", fontsize=14)

    delay_angular_vel_axes[1].set_title("Actual Y Angular Velocity vs Delay compensation Y Angular Velocity")
    delay_angular_vel_axes[1].set_ylabel("Omega Y [rad/s]", fontsize=14)

    delay_angular_vel_axes[2].set_title("Actual Z Angular Velocity vs Delay compensation Z Angular Velocity")
    delay_angular_vel_axes[2].set_ylabel("Omega Z [rad/s]", fontsize=14)
    
    #delay_angular_vel_axes[0].set_xlim([0, 10])
    
    for index in range(len(delay_angle_axes)):
        delay_angular_vel_axes[index].set_xlabel("Time [s]", fontsize=14)
        delay_angular_vel_axes[index].legend(loc='upper right')
        delay_angular_vel_axes[index].grid()
        if ylim_flag:
            delay_angular_vel_axes[index].set_ylim([-0.8, 0.8])

    # Cartesian velocities
    delay_cartesian_vel_fig, delay_cartesian_vel_axes = plt.subplots(3, figsize=delay_fig_size, dpi=save_dpi)
    
    delay_cartesian_vel_axes[0].plot(dataframe["t_sim"], dataframe["vel_x"], label=r"$vel_x$", color=[1.0, 0.0, 1.0], linewidth=linewidth)
    delay_cartesian_vel_axes[1].plot(dataframe["t_sim"], dataframe["vel_y"], label=r"$vel_y$", color=[0.5, 0.0, 1.0], linewidth=linewidth)
    delay_cartesian_vel_axes[2].plot(dataframe["t_sim"], dataframe["vel_z"], label=r"$vel_z$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

    delay_cartesian_vel_axes[0].plot(dataframe["t_sim"][1:], [float(x[9]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$vel_x$-compensated (shifted)", linewidth=linewidth)
    delay_cartesian_vel_axes[1].plot(dataframe["t_sim"][1:], [float(x[10]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$vel_y$-compensated (shifted)", linewidth=linewidth)
    delay_cartesian_vel_axes[2].plot(dataframe["t_sim"][1:], [float(x[11]) for x in [state.split("|") for state in dataframe["P_param_full"][:-1]]], label=r"$vel_z$-compensated (shifted)", linewidth=linewidth)

    delay_cartesian_vel_axes[0].set_title("Actual X Cartesian Velocity vs Delay compensation X Cartesian Velocity")
    delay_cartesian_vel_axes[0].set_ylabel("Vel X [m/s]", fontsize=14)

    delay_cartesian_vel_axes[1].set_title("Actual Y Cartesian Velocity vs Delay compensation Y Cartesian Velocity")
    delay_cartesian_vel_axes[1].set_ylabel("Vel Y [m/s]", fontsize=14)

    delay_cartesian_vel_axes[2].set_title("Actual Z Cartesian Velocity vs Delay compensation Z Cartesian Velocity")
    delay_cartesian_vel_axes[2].set_ylabel("Vel Z [m/s]", fontsize=14)
    
    #delay_angular_vel_axes[0].set_xlim([0, 10])
    
    for index in range(len(delay_angle_axes)):
        delay_cartesian_vel_axes[index].set_xlabel("Time [s]", fontsize=14)
        delay_cartesian_vel_axes[index].legend(loc='upper right')
        delay_cartesian_vel_axes[index].grid()
        #delay_cartesian_vel_axes[index].set_ylim([-0.8, 0.8])

else:
    print("Delay flag is set to False, skipping delay compensation plots.")

print("Generating angle plots...")

angle_ax.plot(dataframe["t_sim"], dataframe["psi_desired"], label=r"$\phi$-,$\theta$-,$\psi$-desired", color="black", linewidth=linewidth)

angle_ax.plot(dataframe["t_sim"], dataframe["phi"], label=r"$\phi$", color=[1.0, 0.0, 1.0], linewidth=linewidth)
angle_ax.plot(dataframe["t_sim"], dataframe["theta"], label=r"$\theta$", color=[0.5, 0.0, 1.0], linewidth=linewidth)
angle_ax.plot(dataframe["t_sim"], dataframe["psi"], label=r"$\psi$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

angle_ax.set_title("Euler Angles in World Frame")
angle_ax.set_ylabel("Euler Angles [rad]", fontsize=14)
#angle_ax.set_xlim([0, 10])
#angle_ax.set_ylim([-0.3, 0.3])
angle_ax.set_xlabel("Time [s]", fontsize=14)
angle_ax.legend(loc='upper right')
angle_ax.grid()

if generate_prediction_plots:
    print("Generating MPC predicted Phi vs. actual Phi plot...")

    # Plot actual vs predicted phi
    phi_fig = plt.figure(figsize=angle_fig_size, dpi=save_dpi)
    phi_ax = phi_fig.add_subplot(111)

    phi_ax.plot(dataframe["t_sim"], dataframe["phi_desired"], label=r"$\phi$-desired", color="black", linewidth=linewidth)
    phi_ax.plot(dataframe["t_sim"], dataframe["phi"], label=r"$\phi$", color=[1.0, 0.0, 1.0], linewidth=linewidth)

    for i in range(0, len(dataframe["t_sim"])):
        if i % plot_every_predicted == 0:
            X_t_raw = dataframe['X_t'][i].split(";")
            X_t = []
            for timestep in X_t_raw:
                state = [float(x) for x in timestep.split("|")]
                X_t.append(state)

            X_t = [item for sublist in X_t for item in sublist] # Flatten
            if delay_flag:
                phi_ax.plot(tuple(numeric_range(dataframe["t_sim"][i] - dt, dataframe["t_sim"][i] - dt + (N + 1) * dt, dt)), X_t[0::n], linewidth=linewidth * 0.75, linestyle='--')
            else:
                phi_ax.plot(tuple(numeric_range(dataframe["t_sim"][i], dataframe["t_sim"][i] + (N + 1) * dt, dt)), X_t[0::n], linewidth=linewidth * 0.75, linestyle='--')


    phi_ax.set_title("Phi (Roll) vs predicted Phi (Roll) in World Frame")
    phi_ax.set_ylabel("Phi (Roll) [rad]", fontsize=14)
    #angle_ax.set_xlim([0, 10])
    #angle_ax.set_ylim([-0.3, 0.3])
    phi_ax.set_xlabel("Time [s]", fontsize=14)
    phi_ax.legend(loc='upper right')
    phi_ax.grid()

    print("Generating MPC predicted Theta vs. actual Theta plot...")

    # Plot actual vs predicted theta
    theta_fig = plt.figure(figsize=angle_fig_size, dpi=save_dpi)
    theta_ax = theta_fig.add_subplot(111)

    theta_ax.plot(dataframe["t_sim"], dataframe["theta_desired"], label=r"$\theta$-desired", color="black", linewidth=linewidth)
    theta_ax.plot(dataframe["t_sim"], dataframe["theta"], label=r"$\theta$", color=[1.0, 0.0, 1.0], linewidth=linewidth)

    for i in range(0, len(dataframe["t_sim"])):
        if i % plot_every_predicted == 0:
            X_t_raw = dataframe['X_t'][i].split(";")
            X_t = []
            for timestep in X_t_raw:
                state = [float(x) for x in timestep.split("|")]
                X_t.append(state)

            X_t = [item for sublist in X_t for item in sublist] # Flatten
            if delay_flag:
                theta_ax.plot(tuple(numeric_range(dataframe["t_sim"][i] - dt, dataframe["t_sim"][i] - dt + (N + 1) * dt, dt)), X_t[1::n], linewidth=linewidth * 0.75, linestyle='--')
            else:
                theta_ax.plot(tuple(numeric_range(dataframe["t_sim"][i], dataframe["t_sim"][i]+ (N + 1) * dt, dt)), X_t[1::n], linewidth=linewidth * 0.75, linestyle='--')

    theta_ax.set_title("Theta (Pitch) vs predicted Theta (Pitch) in World Frame")
    theta_ax.set_ylabel("Theta (Pitch) [rad]", fontsize=14)
    #angle_ax.set_xlim([0, 10])
    #angle_ax.set_ylim([-0.3, 0.3])
    theta_ax.set_xlabel("Time [s]", fontsize=14)
    theta_ax.legend(loc='upper right')
    theta_ax.grid()

    print("Generating MPC predicted Psi vs. actual Psi plot...")

    # Plot actual vs predicted psi
    psi_fig = plt.figure(figsize=angle_fig_size, dpi=save_dpi)
    psi_ax = psi_fig.add_subplot(111)

    psi_ax.plot(dataframe["t_sim"], dataframe["psi_desired"], label=r"$\psi$-desired", color="black", linewidth=linewidth)
    psi_ax.plot(dataframe["t_sim"], dataframe["psi"], label=r"$\psi$", color=[1.0, 0.0, 1.0], linewidth=linewidth)

    for i in range(0, len(dataframe["t_sim"])):
        if i % plot_every_predicted == 0:
            X_t_raw = dataframe['X_t'][i].split(";")
            X_t = []
            for timestep in X_t_raw:
                state = [float(x) for x in timestep.split("|")]
                X_t.append(state)

            X_t = [item for sublist in X_t for item in sublist] # Flatten
            if delay_flag:
                psi_ax.plot(tuple(numeric_range(dataframe["t_sim"][i] - dt, dataframe["t_sim"][i] - dt + (N + 1) * dt, dt)), X_t[2::n], linewidth=linewidth * 0.75, linestyle='--')
            else:
                psi_ax.plot(tuple(numeric_range(dataframe["t_sim"][i], dataframe["t_sim"][i] + (N + 1) * dt, dt)), X_t[2::n], linewidth=linewidth * 0.75, linestyle='--')

    psi_ax.set_title("Psi (Yaw) vs predicted Psi (Yaw) in World Frame")
    psi_ax.set_ylabel("Psi (Yaw) [rad]", fontsize=14)
    #angle_ax.set_xlim([0, 10])
    #angle_ax.set_ylim([-0.3, 0.3])
    psi_ax.set_xlabel("Time [s]", fontsize=14)
    psi_ax.legend(loc='upper right')
    psi_ax.grid()

else:
    print("generate_prediction_plots = False, skipping...")

print("Generating position plot...")

# pos_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
# pos_ax = pos_fig.add_subplot(111)

pos_fig, (pos_ax1, pos_ax2) = plt.subplots(2, 1, sharex=True, figsize=pos_fig_size, dpi=save_dpi)

pos_ax1.plot(dataframe["t_sim"], dataframe["pos_x_desired"], color=[0.0, 0.5, 1.0], linewidth=linewidth)
pos_ax1.plot(dataframe["t_sim"], dataframe["pos_z_desired"], label=r"$p_{{z}_{desired}}$", color=[0.3, 0.3, 0.3], linewidth=linewidth)

pos_ax2.plot(dataframe["t_sim"], dataframe["pos_x_desired"], label=r"$p_{{x,y}_{desired}}$", color="black", linewidth=linewidth)
pos_ax2.plot(dataframe["t_sim"], dataframe["pos_z_desired"], color=[0.3, 0.3, 0.3], linewidth=linewidth)

pos_ax1.plot(dataframe["t_sim"], dataframe["pos_x"], color=[1.0, 0.0, 1.0], linewidth=linewidth)
pos_ax1.plot(dataframe["t_sim"], dataframe["pos_y"], color=[0.5, 0.0, 1.0], linewidth=linewidth)
pos_ax1.plot(dataframe["t_sim"], dataframe["pos_z"], label=r"$p_z$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

pos_ax2.plot(dataframe["t_sim"], dataframe["pos_x"], color=[1.0, 0.0, 1.0], label=r"$p_x$", linewidth=linewidth)
pos_ax2.plot(dataframe["t_sim"], dataframe["pos_y"], color=[0.5, 0.0, 1.0], label=r"$p_y$", linewidth=linewidth)
pos_ax2.plot(dataframe["t_sim"], dataframe["pos_z"], color=[0.0, 0.0, 1.0], linewidth=linewidth)

pos_ax1.set_title("Cartesian Position in World Frame")
pos_ax2.set_ylabel("Position [m]", fontsize=14)
pos_ax2.yaxis.set_label_coords(-0.08,1.1)
pos_ax2.set_xlabel("Time [s]", fontsize=14)

#pos_ax1.set_xlim([0, 10])
#pos_ax2.set_xlim([0, 10])

pos_ax1.set_ylim([1, 1.2])
pos_ax2.set_ylim([-0.15, 0.15])
pos_ax1.legend(loc='upper right')
pos_ax2.legend(loc='lower right')
pos_ax1.grid()
pos_ax2.grid()

pos_ax1.spines['bottom'].set_visible(False)
pos_ax2.spines['top'].set_visible(False)
pos_ax1.xaxis.tick_top()
pos_ax1.tick_params(labeltop=False)  # don't put tick labels at the top
pos_ax2.xaxis.tick_bottom()

d = .01  # how big to make the diagonal lines in axes coordinates
kwargs = dict(transform=pos_ax1.transAxes, color='k', clip_on=False)
pos_ax1.plot((-d, +d), (-d, +d), **kwargs)        # top-left diagonal
pos_ax1.plot((1 - d, 1 + d), (-d, +d), **kwargs)  # top-right diagonal

kwargs.update(transform=pos_ax2.transAxes) # switch to the bottom axes
pos_ax2.plot((-d, +d), (1 - d, 1 + d), **kwargs) # bottom-left diagonal
pos_ax2.plot((1 - d, 1 + d), (1 - d, 1 + d), **kwargs) # bottom-right diagonal

print("Generating reaction force plot...")

force_fig = plt.figure(figsize=force_fig_size, dpi=save_dpi)
force_ax = force_fig.add_subplot(111)

force_ax.plot(dataframe["t_sim"], dataframe["f_x_left"], label=r"$f_{{left}_x}$", color=[1.0, 0.0, 1.0], linewidth=linewidth)
force_ax.plot(dataframe["t_sim"], dataframe["f_y_left"], label=r"$f_{{left}_y}$", color=[0.5, 0.0, 1.0], linewidth=linewidth)
force_ax.plot(dataframe["t_sim"], dataframe["f_z_left"], label=r"$f_{{left}_z}$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

force_ax.plot(dataframe["t_sim"], dataframe["f_x_right"], label=r"$f_{{right}_x}$", color=[0.0, 0.5, 1.0], linewidth=linewidth)
force_ax.plot(dataframe["t_sim"], dataframe["f_y_right"], label=r"$f_{{right}_y}$", color=[0.0, 1.0, 1.0], linewidth=linewidth)
force_ax.plot(dataframe["t_sim"], dataframe["f_z_right"], label=r"$f_{{right}_z}$", color=[0.3, 0.3, 0.3], linewidth=linewidth)

force_ax.set_title("Contact Forces in World Frame")
force_ax.set_ylabel("Forces [N]", fontsize=14)
#force_ax.set_xlim([0, 10])
force_ax.set_ylim([-300, 1000])
force_ax.set_xlabel("Time [s]", fontsize=14)
force_ax.legend(loc='upper right')
force_ax.grid()

kf_error_fig = plt.figure(figsize=force_fig_size, dpi=save_dpi)
kf_error_ax = kf_error_fig.add_subplot(111)

kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_phi"], label=r"kf_error_phi", linewidth=linewidth)
kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_theta"], label=r"kf_error_theta", linewidth=linewidth)
kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_psi"], label=r"kf_error_psi", linewidth=linewidth)

kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_pos_x"], label=r"kf_error_pos_x", linewidth=linewidth)
kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_pos_y"], label=r"kf_error_pos_y", linewidth=linewidth)
kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_pos_z"], label=r"kf_error_pos_z", linewidth=linewidth)

kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_omega_x"], label=r"kf_error_omega_x", linewidth=linewidth)
kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_omega_y"], label=r"kf_error_omega_y", linewidth=linewidth)
kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_omega_z"], label=r"kf_error_omega_z", linewidth=linewidth)

kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_vel_x"], label=r"kf_error_vel_x", linewidth=linewidth)
kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_vel_y"], label=r"kf_error_vel_y", linewidth=linewidth)
kf_error_ax.plot(dataframe["t_sim"], dataframe["kf_error_vel_z"], label=r"kf_error_vel_z", linewidth=linewidth)

kf_error_ax.set_title("Kalman Filter Error")
kf_error_ax.set_ylabel("Kalman Filter Error", fontsize=14)

kf_error_ax.set_xlabel("Time [s]", fontsize=14)
kf_error_ax.legend(loc='upper right')
kf_error_ax.grid()

print("Max full iteration time excluding first iteration (fix that!):", max(dataframe['full_iteration_time'][1:]), "[ms]")
print("Average full iteration time excluding first iteration (fix that!):", np.mean(dataframe['full_iteration_time'][1:]), "[ms]")

print("Saving plots to pdf in dir:", plot_image_dir)

# Save plots
angle_fig.savefig(plot_image_dir + "angles.pdf", dpi=save_dpi, bbox_inches='tight')
force_fig.savefig(plot_image_dir + "forces.pdf", dpi=save_dpi, bbox_inches='tight')
kf_error_fig.savefig(plot_image_dir + "kf_errors.pdf", dpi=save_dpi, bbox_inches='tight')
pos_fig.savefig(plot_image_dir + "position.pdf", dpi=save_dpi, bbox_inches='tight')

if generate_prediction_plots:
    phi_fig.savefig(plot_image_dir + "phi_actual_vs_mpc.pdf", dpi=save_dpi, bbox_inches='tight')
    theta_fig.savefig(plot_image_dir + "theta_actual_vs_mpc.pdf", dpi=save_dpi, bbox_inches='tight')
    psi_fig.savefig(plot_image_dir + "psi_actual_vs_mpc.pdf", dpi=save_dpi, bbox_inches='tight')

if delay_flag:
    delay_angle_fig.savefig(plot_image_dir + "angles_actual_vs_compensated.pdf", dpi=save_dpi, bbox_inches='tight')
    delay_pos_fig.savefig(plot_image_dir + "position_actual_vs_compensated.pdf", dpi=save_dpi, bbox_inches='tight')
    delay_angular_vel_fig.savefig(plot_image_dir + "angular_vel_actual_vs_compensated.pdf", dpi=save_dpi, bbox_inches='tight')
    delay_cartesian_vel_fig.savefig(plot_image_dir + "cartesian_vel_actual_vs_compensated.pdf", dpi=save_dpi, bbox_inches='tight')

# print("Raw data:\n", dataframe)

rms_error_vector = np.zeros((n-1, 1))

actual_states = dataframe.iloc[:, range(2, 2+12)].values # Rows are iterations, columns are states
reference_states = dataframe.iloc[:, range(14, 14+12)].values

data_length = int(len(dataframe["t_sim"]) * 0.95)

for t_index in range(data_length):
    for state_index in range(n-1):
        rms_error_vector[state_index] += (reference_states[t_index, state_index] - actual_states[t_index, state_index])**2

rms_error_vector /= data_length

rms_error_vector = [math.sqrt(x) for x in rms_error_vector]

rms_dataframe = pd.DataFrame([tuple(rms_error_vector)], columns=["phi", "theta", "psi", "pos_x", "pos_y", "pos_z", "omega_x", "omega_y", "omega_z", "vel_x", "vel_y", "vel_z"])

print(rms_dataframe)

print("RMS Error Vector x_ref - x_actual:", rms_error_vector)

rms_error_vector = np.zeros((n-1, 1))

actual_states = dataframe.iloc[:, range(2, 2+12)].values # Rows are iterations, columns are states
kf_states = dataframe.iloc[:, range(109, 109+12)].values

# print(kf_states)

for t_index in range(data_length):
    for state_index in range(n-1):
        rms_error_vector[state_index] += (actual_states[t_index, state_index] - kf_states[t_index, state_index])**2

rms_error_vector /= data_length

rms_error_vector = [math.sqrt(x) for x in rms_error_vector]

rms_dataframe = pd.DataFrame([tuple(rms_error_vector)], columns=["phi", "theta", "psi", "pos_x", "pos_y", "pos_z", "omega_x", "omega_y", "omega_z", "vel_x", "vel_y", "vel_z"])

print(rms_dataframe)

print("RMS Error Vector x_actual - x_kf:", rms_error_vector)

if show_plots:
    print("Showing plots.")
    plt.show()