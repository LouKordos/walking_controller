import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

home_dir = os.environ['HOME']

filename = home_dir + "/dev/walking_controller/plot_data/mpc_log.csv"
plot_image_dir = home_dir + "/Pictures/matplotlib_plots/mpc_log/"

dataframe = pd.read_csv(filename)

scaling_factor = 0.75
save_dpi = 500 * scaling_factor

relative_width_pos = 5 # inches I think
relative_height_pos = 50 / 25.4 # inches I think

relative_width_angle = 5 # inches I think
relative_height_angle = 50 / 25.4 # inches I think

relative_width_force = 10 # inches I think
relative_height_force = 55 / 25.4 # inches I think

angle_fig_size = (relative_width_angle * (1/scaling_factor), relative_height_angle * (1/scaling_factor))
pos_fig_size = (relative_width_pos * (1/scaling_factor), relative_height_pos * (1/scaling_factor))
force_fig_size = (relative_width_force * (1/scaling_factor), relative_height_force * (1/scaling_factor))

angle_fig = plt.figure(figsize=angle_fig_size, dpi=save_dpi)
angle_ax = angle_fig.add_subplot(111)

linewidth = 0.8

angle_ax.plot(dataframe['t'], dataframe["psi_desired"], label=r"$\phi$-,$\theta$-,$\psi$-desired", color="black", linewidth=linewidth)

angle_ax.plot(dataframe['t'], dataframe["phi"], label=r"$\phi$", color=[1.0, 0.0, 1.0], linewidth=linewidth)
angle_ax.plot(dataframe['t'], dataframe["theta"], label=r"$\theta$", color=[0.5, 0.0, 1.0], linewidth=linewidth)
angle_ax.plot(dataframe['t'], dataframe["psi"], label=r"$\psi$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

angle_ax.set_title("Euler Angles in World Frame")
angle_ax.set_ylabel("Euler Angles [rad]", fontsize=14)
angle_ax.set_xlim([0, 10])
angle_ax.set_ylim([-0.15, 0.15])
angle_ax.set_xlabel("Time [s]", fontsize=14)
angle_ax.legend(loc='upper right')

# pos_fig = plt.figure(figsize=save_fig_size, dpi=save_dpi)
# pos_ax = pos_fig.add_subplot(111)

pos_fig, (pos_ax1, pos_ax2) = plt.subplots(2, 1, sharex=True, figsize=pos_fig_size, dpi=save_dpi)

pos_ax1.plot(dataframe['t'], dataframe["pos_x_desired"], color=[0.0, 0.5, 1.0], linewidth=linewidth)
pos_ax1.plot(dataframe['t'], dataframe["pos_z_desired"], label=r"$p_{{z}_{desired}}$", color=[0.3, 0.3, 0.3], linewidth=linewidth)

pos_ax2.plot(dataframe['t'], dataframe["pos_x_desired"], label=r"$p_{{x,y}_{desired}}$", color="black", linewidth=linewidth)
pos_ax2.plot(dataframe['t'], dataframe["pos_z_desired"], color=[0.3, 0.3, 0.3], linewidth=linewidth)

pos_ax1.plot(dataframe['t'], dataframe["pos_x"], color=[1.0, 0.0, 1.0], linewidth=linewidth)
pos_ax1.plot(dataframe['t'], dataframe["pos_y"], color=[0.5, 0.0, 1.0], linewidth=linewidth)
pos_ax1.plot(dataframe['t'], dataframe["pos_z"], label=r"$p_z$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

pos_ax2.plot(dataframe['t'], dataframe["pos_x"], color=[1.0, 0.0, 1.0], label=r"$p_x$", linewidth=linewidth)
pos_ax2.plot(dataframe['t'], dataframe["pos_y"], color=[0.5, 0.0, 1.0], label=r"$p_y$", linewidth=linewidth)
pos_ax2.plot(dataframe['t'], dataframe["pos_z"], color=[0.0, 0.0, 1.0], linewidth=linewidth)



pos_ax1.set_title("Cartesian Position in World Frame")
pos_ax2.set_ylabel("Position [m]", fontsize=14)
pos_ax2.yaxis.set_label_coords(-0.08,1.1)
pos_ax2.set_xlabel("Time [s]", fontsize=14)

pos_ax1.set_xlim([0, 10])
pos_ax2.set_xlim([0, 10])

pos_ax1.set_ylim([1, 1.2])
pos_ax2.set_ylim([-0.15, 0.15])
pos_ax1.legend(loc='upper right')
pos_ax2.legend(loc='lower right')

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

force_fig = plt.figure(figsize=force_fig_size, dpi=save_dpi)
force_ax = force_fig.add_subplot(111)

force_ax.plot(dataframe['t'], dataframe["f_x_left"], label=r"$f_{{left}_x}$", color=[1.0, 0.0, 1.0], linewidth=linewidth)
force_ax.plot(dataframe['t'], dataframe["f_y_left"], label=r"$f_{{left}_y}$", color=[0.5, 0.0, 1.0], linewidth=linewidth)
force_ax.plot(dataframe['t'], dataframe["f_z_left"], label=r"$f_{{left}_z}$", color=[0.0, 0.0, 1.0], linewidth=linewidth)

force_ax.plot(dataframe['t'], dataframe["f_x_right"], label=r"$f_{{right}_x}$", color=[0.0, 0.5, 1.0], linewidth=linewidth)
force_ax.plot(dataframe['t'], dataframe["f_y_right"], label=r"$f_{{right}_y}$", color=[0.0, 1.0, 1.0], linewidth=linewidth)
force_ax.plot(dataframe['t'], dataframe["f_z_right"], label=r"$f_{{right}_z}$", color=[0.3, 0.3, 0.3], linewidth=linewidth)

force_ax.set_title("Contact Forces in World Frame")
force_ax.set_ylabel("Forces [N]", fontsize=14)
force_ax.set_xlim([0, 10])
force_ax.set_ylim([-300, 1000])
force_ax.set_xlabel("Time [s]", fontsize=14)
force_ax.legend(loc='upper right')

angle_fig.savefig(plot_image_dir + "angles.pdf", dpi=save_dpi, bbox_inches='tight')
force_fig.savefig(plot_image_dir + "forces.pdf", dpi=save_dpi, bbox_inches='tight')
pos_fig.savefig(plot_image_dir + "position.pdf", dpi=save_dpi, bbox_inches='tight')

print(dataframe)