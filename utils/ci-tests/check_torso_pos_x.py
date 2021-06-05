import pandas as pd
import numpy as np
import sys

mpc_log = pd.read_csv("/plot_data/1_mpc_log.csv")

min_allowed_torso_pos_x = float(sys.argv[1])
max_allowed_torso_pos_x = float(sys.argv[2])

print("Check if last recorded Torso X position is >", min_allowed_torso_pos_x, "and <", max_allowed_torso_pos_x)

for i in range(len(mpc_log["t_sim"])):
    if mpc_log["pos_x"] < min_allowed_torso_pos_x or mpc_log["pos_x"] > max_allowed_torso_pos_x:
        sys.exit(1)

sys.exit(0)