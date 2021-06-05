import pandas as pd
import numpy as np
import sys

mpc_log = pd.read_csv("/home/runner/work/plot_data/1_mpc_log.csv")

min_allowed_torso_pos_x = float(sys.argv[1])
max_allowed_torso_pos_x = float(sys.argv[2])

print("Check if last recorded Torso X position is >", min_allowed_torso_pos_x, "and <", max_allowed_torso_pos_x)

if mpc_log["pos_x"][-1] < min_allowed_torso_pos_x or mpc_log["pos_x"][-1] > max_allowed_torso_pos_x:
    print("Torso Pos X with value", mpc_log["pos_X"][-1], "outside bounds, exiting with error code 1.")
    sys.exit(1)

print("Last Pos X within bounds, test passed.")
sys.exit(0)