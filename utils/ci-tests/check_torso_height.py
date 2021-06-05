import pandas as pd
import numpy as np
import sys

mpc_log = pd.read_csv("/home/runner/work/plot_data/1_mpc_log.csv")

min_allowed_torso_height = float(sys.argv[1])
max_allowed_torso_height = float(sys.argv[2])

print("Checking if all recorded torso heights are >", min_allowed_torso_height, "and <", max_allowed_torso_height)

for i in range(len(mpc_log["t_sim"])):
    if mpc_log["pos_z"][i] < min_allowed_torso_height or mpc_log["pos_z"][i] > max_allowed_torso_height:
        print("Torso height with value", mpc_log["pos_z"][i], "outside bounds, exiting with error code 1.")
        sys.exit(1)

print("All Torso height values within bounds, test passed.")
sys.exit(0)