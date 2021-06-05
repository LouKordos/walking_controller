import pandas as pd
import numpy as np
import sys

mpc_log = pd.read_csv("/home/runner/work/plot_data/1_mpc_log.csv")

min_allowed_torso_pos_y = float(sys.argv[1])
max_allowed_torso_pos_y = float(sys.argv[2])

print("Check if last recorded Torso Y position is >", min_allowed_torso_pos_y, "and <", max_allowed_torso_pos_y)

if mpc_log["pos_y"][-1] < min_allowed_torso_pos_y or mpc_log["pos_y"][-1] > max_allowed_torso_pos_y:
    print("Torso Pos Y with value", mpc_log["pos_y"][-1], "outside bounds, exiting with error code 1.")
    sys.exit(1)

print("Last Pos Y within bounds, test passed.")
sys.exit(0)