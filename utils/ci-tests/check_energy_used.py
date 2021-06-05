import pandas as pd
import numpy as np
import sys

min_allowed_energy = float(sys.argv[1])
max_allowed_energy = float(sys.argv[2])

left_leg_log = pd.read_csv("/home/runner/work/plot_data/1_left.csv")
right_leg_log = pd.read_csv("/home/runner/work/plot_data/1_right.csv")

data_length = int(len(left_leg_log["t_sim"]) * 0.98)

combined_power = []
for i in range(data_length):
    combined_power_t = 0
    for joint_index in range(1, 5):
        combined_power_t += abs(left_leg_log[f"tau_{joint_index}"][i] * left_leg_log[f'theta{joint_index}_dot'][i])
        combined_power_t += abs(right_leg_log[f"tau_{joint_index}"][i] * right_leg_log[f'theta{joint_index}_dot'][i])
    combined_power.append(combined_power_t)

print("Average power consumption of both legs:", np.mean(combined_power), "[W]")

total_energy = np.sum(combined_power) * (1.0/1000.0) / 3600.0

print("Total energy consumed by leg in", left_leg_log['t_sim'][data_length], "seconds:", total_energy, "[Wh]")

if total_energy > max_allowed_energy or total_energy < min_allowed_energy:
    print("Energy consumption outside allowed range.")
    sys.exit(1)
else:
    sys.exit(0)