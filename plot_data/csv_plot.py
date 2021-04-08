import numpy as np
import matplotlib.pyplot as plt
import os
import threading

'''
<< "t,"
<< "theta1,theta2,theta3,theta4,theta5,theta1_dot,theta2_dot,theta3_dot,theta4_dot,theta5_dot,"
<< "tau_1,tau_2,tau_3,tau_4,tau_5,"
<< "foot_pos_x,foot_pos_y,foot_pos_z,"
<< "foot_pos_x_desired,foot_pos_y_desired,foot_pos_z_desired"
<< "foot_vel_x,foot_vel_y,foot_vel_z,"
<< "foot_vel_x_desired,foot_vel_y_desired,foot_vel_z_desired" << std::endl;
'''

home_dir = os.environ['HOME']

filenames = os.listdir(home_dir + "/dev/walking_controller/plot_data/")

largest_index = 0

for name in filenames:
	try:
		index = int(name.split('.')[0].replace('_left', '').replace('_right', ''))
		if index > largest_index:
			largest_index = index
	except:
		print("Invalid parse with filename:", name)

filename_left = str(largest_index) + "_left.csv"
filename_right = str(largest_index) + "_right.csv"
print("filename_left:", filename_left)

def plot_file_data(filename):

	data = np.genfromtxt(filename, delimiter=',', skip_header=50,
	     	skip_footer=0, names=['t', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta1_dot', 'theta2_dot', 'theta3_dot', 'theta4_dot', 'theta5_dot', 'tau_1', 'tau_2', 'tau_3', 'tau_4', 'tau_5', 'foot_pos_x', 'foot_pos_y', 'foot_pos_z', 'foot_pos_x_desired', 'foot_pos_y_desired', 'foot_pos_z_desired', 'foot_vel_x', 'foot_vel_y', 'foot_vel_z', 'foot_vel_x_desired', 'foot_vel_y_desired', 'foot_vel_z_desired', 'foot_phi', 'foot_theta', 'foot_psi', 'foot_phi_desired', 'foot_theta_desired', 'foot_psi_desired', 'current_trajectory_time'])

	#print(data)
	scaling_factor = 0.75

	save_dpi = 190 * scaling_factor
	save_fig_size = (10 * (1/scaling_factor), 5 * (1/scaling_factor))

	x_pos_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
	x_pos_ax = x_pos_fig.add_subplot(111)
	x_pos_ax.set_ylabel('Measured and desired X foot position [m]')
	x_pos_ax.set_xlabel('time [s]')
	x_pos_ax.set_title('X Position')

	x_pos_ax.plot(data['t'], data['foot_pos_x'], label='foot_position_x')
	x_pos_ax.plot(data['t'], data['foot_pos_x_desired'], label='foot_position_x_desired')
	plt.legend()

	y_pos_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
	y_pos_ax = y_pos_fig.add_subplot(111)
	y_pos_ax.set_ylabel('Measured and desired Y foot position [m]')
	y_pos_ax.set_xlabel('time [s]')
	y_pos_ax.set_title('Y Position')

	y_pos_ax.plot(data['t'], data['foot_pos_y'], label='foot_position_y')
	y_pos_ax.plot(data['t'], data['foot_pos_y_desired'], label='foot_position_y_desired')
	plt.legend()

	z_pos_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
	z_pos_ax = z_pos_fig.add_subplot(111)
	z_pos_ax.set_ylabel('Measured and desired Z foot position [m]')
	z_pos_ax.set_xlabel('time [s]')
	z_pos_ax.set_title('Z Position')

	z_pos_ax.plot(data['t'], data['foot_pos_z'], label='foot_position_z')
	z_pos_ax.plot(data['t'], data['foot_pos_z_desired'], label='foot_position_z_desired')
	plt.legend()

	x_vel_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
	x_vel_ax = x_vel_fig.add_subplot(111)
	x_vel_ax.set_ylabel('Measured and desired X foot velocity [m/s]')
	x_vel_ax.set_xlabel('time [s]')
	x_vel_ax.set_title('X Velocity')

	x_vel_ax.plot(data['t'], data['foot_vel_x'], label='foot_velocity_x')
	x_vel_ax.plot(data['t'], data['foot_vel_x_desired'], label='foot_velocity_x_desired')
	plt.legend()

	y_vel_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
	y_vel_ax = y_vel_fig.add_subplot(111)
	y_vel_ax.set_ylabel('Measured and desired Y foot velocity [m/s]')
	y_vel_ax.set_xlabel('time [s]')
	y_vel_ax.set_title('Y Velocity')

	y_vel_ax.plot(data['t'], data['foot_vel_y'], label='foot_velocity_y')
	y_vel_ax.plot(data['t'], data['foot_vel_y_desired'], label='foot_velocity_y_desired')
	plt.legend()

	z_vel_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
	z_vel_ax = z_vel_fig.add_subplot(111)
	z_vel_ax.set_ylabel('Measured and desired Z foot velocity [m/s]')
	z_vel_ax.set_xlabel('time [s]')
	z_vel_ax.set_title('Z Velocity')

	z_vel_ax.plot(data['t'], data['foot_vel_z'], label='foot_velocity_z')
	z_vel_ax.plot(data['t'], data['foot_vel_z_desired'], label='foot_velocity_z_desired')
	plt.legend()

	torque_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
	torque_ax = torque_fig.add_subplot(111)
	torque_ax.set_ylabel('Joint torque [Nm]')
	torque_ax.set_xlabel('time [s]')
	torque_ax.set_title('Joint torques')

	torque_ax.plot(data['t'], data['tau_1'], label='tau_1')
	torque_ax.plot(data['t'], data['tau_2'], label='tau_2')
	torque_ax.plot(data['t'], data['tau_3'], label='tau_3')
	torque_ax.plot(data['t'], data['tau_4'], label='tau_4')
	torque_ax.plot(data['t'], data['tau_5'], label='tau_5')
	plt.legend()

	power_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
	power_ax = power_fig.add_subplot(111)

	# print(np.sum([data[f"tau_{i}"] * data[f'theta{i}_dot'] for i in range(1, 5)]))

	combined_power = []
	for i in range(len(data['tau_1'])):
		combined_power_t = 0
		for joint_index in range(1, 5):
			combined_power_t += abs(data[f"tau_{joint_index}"][i] * data[f'theta{joint_index}_dot'][i])
		combined_power.append(combined_power_t)
		#print(combined_power_t)

	# print(combined_power)

	print("Average leg power:", np.mean(combined_power), "[W]")

	print("Total energy consumed by leg in", data['t'][-1], "seconds:", np.sum(combined_power) * (1/1000) / 3600, "[Wh]")

	power_ax.plot(data['t'], combined_power, label="Combined leg power")
	plt.legend()

	print("Finished plotting")

	if('left' in filename):
		x_pos_fig.savefig(home_dir + '/Pictures/matplotlib_pics/left/foot_pos_x.pdf', dpi=save_dpi, bbox_inches='tight')
		y_pos_fig.savefig(home_dir + '/Pictures/matplotlib_pics/left/foot_pos_y.pdf', dpi=save_dpi, bbox_inches='tight')
		z_pos_fig.savefig(home_dir + '/Pictures/matplotlib_pics/left/foot_pos_z.pdf', dpi=save_dpi, bbox_inches='tight')

		x_vel_fig.savefig(home_dir + '/Pictures/matplotlib_pics/left/foot_vel_x.pdf', dpi=save_dpi, bbox_inches='tight')
		y_vel_fig.savefig(home_dir + '/Pictures/matplotlib_pics/left/foot_vel_y.pdf', dpi=save_dpi, bbox_inches='tight')
		z_vel_fig.savefig(home_dir + '/Pictures/matplotlib_pics/left/foot_vel_z.pdf', dpi=save_dpi, bbox_inches='tight')

		torque_fig.savefig(home_dir + '/Pictures/matplotlib_pics/left/torques.pdf', dpi=save_dpi, bbox_inches='tight')
		power_fig.savefig(home_dir + '/Pictures/matplotlib_pics/left/combined_power.pdf', dpi=save_dpi, bbox_inches='tight')

	else:
		x_pos_fig.savefig(home_dir + '/Pictures/matplotlib_pics/right/foot_pos_x.pdf', dpi=save_dpi, bbox_inches='tight')
		y_pos_fig.savefig(home_dir + '/Pictures/matplotlib_pics/right/foot_pos_y.pdf', dpi=save_dpi, bbox_inches='tight')
		z_pos_fig.savefig(home_dir + '/Pictures/matplotlib_pics/right/foot_pos_z.pdf', dpi=save_dpi, bbox_inches='tight')

		x_vel_fig.savefig(home_dir + '/Pictures/matplotlib_pics/right/foot_vel_x.pdf', dpi=save_dpi, bbox_inches='tight')
		y_vel_fig.savefig(home_dir + '/Pictures/matplotlib_pics/right/foot_vel_y.pdf', dpi=save_dpi, bbox_inches='tight')
		z_vel_fig.savefig(home_dir + '/Pictures/matplotlib_pics/right/foot_vel_z.pdf', dpi=save_dpi, bbox_inches='tight')

		torque_fig.savefig(home_dir + '/Pictures/matplotlib_pics/right/torques.pdf', dpi=save_dpi, bbox_inches='tight')
		power_fig.savefig(home_dir + '/Pictures/matplotlib_pics/right/combined_power.pdf', dpi=save_dpi, bbox_inches='tight')

	print("Finished saving")

# t1 = threading.Thread(target=plot_file_data, args=(filename_left, )) 
# t2 = threading.Thread(target=plot_file_data, args=(filename_right,)) 

# t1.start()
# t2.start() 

# t1.join()
# t2.join()
plot_file_data(filename_left)
plot_file_data(filename_right)

print("Done.")