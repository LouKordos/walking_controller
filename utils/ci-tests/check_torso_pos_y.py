import pandas as pd
import numpy as np
import sys

mpc_log = pd.read_csv("/plot_data/1_mpc_log.csv")

min_allowed_torso_pos_y = float(sys.argv[1])
max_allowed_torso_pos_y = float(sys.argv[2])

print("Check if last recorded Torso Y position is >", min_allowed_torso_pos_y, "and <", max_allowed_torso_pos_y)

for i in range(len(mpc_log["t_sim"])):
    if mpc_log["pos_y"] < min_allowed_torso_pos_y or mpc_log["pos_y"] > max_allowed_torso_pos_y:
        sys.exit(1)

sys.exit(0)