#!/usr/bin/env python3

import odrive
import time
from odrive.enums import *
from odrive.utils import dump_errors

odrv0 = odrive.find_any()

bat_n_cells = 6
bat_capacity = 5
Kv = 70
current_limit = 60
calibration_current = 40
pole_pairs = 15
requested_current_range = 90

def save_config(odrv):
    try:
        odrv.save_configuration()
    except:
        print("This error is most likely expected.")

def erase_config(odrv):
    try:
        odrv.erase_configuration()
    except:
        print("This error is most likely expected.")

print("Erasing configuration...")

erase_config(odrv0)

time.sleep(2)

odrv0 = odrive.find_any()

odrv0.config.dc_bus_undervoltage_trip_level = 3.3 * bat_n_cells

odrv0.config.dc_bus_overvoltage_trip_level = 4.25 * bat_n_cells

odrv0.config.dc_max_positive_current = bat_capacity * 10

odrv0.config.dc_max_negative_current = -bat_capacity * 1.5

odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT

odrv0.axis0.motor.config.pole_pairs = pole_pairs

odrv0.axis0.motor.config.torque_constant = 8.27 / Kv

odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION

while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

dump_errors(odrv0)

print("Check for errors above!")

odrv0.axis0.motor.config.pre_calibrated = True

odrv0.axis0.motor.config.current_lim = current_limit

odrv0.axis0.motor.config.calibration_current = calibration_current

odrv0.axis0.motor.config.requested_current_range = requested_current_range

save_config(odrv0)

time.sleep(2)

odrv0 = odrive.find_any()

odrv0.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0

odrv0.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0

save_config(odrv0)

time.sleep(2)

odrv0 = odrive.find_any()

odrv0.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION

while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

save_config(odrv0)

time.sleep(2)

odrv0 = odrive.find_any()

odrv0.axis0.controller.config.vel_integrator_gain = 0.2
odrv0.axis0.controller.config.vel_gain = 0.5
odrv0.axis0.controller.config.pos_gain = 25

odrv0.axis0.controller.config.vel_limit = 10

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
time.sleep(2)

save_config(odrv0)

odrv0 = odrive.find_any()

if input("Configuration finished, run closed loop movements now? (y/n)") == "y":
    odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.input_pos = 20
    time.sleep(3)
    odrv0.axis0.controller.input_pos = -20
    time.sleep(3)
else:
    print("Not running closed loop control.")

odrv0.axis0.requested_state = AXIS_STATE_IDLE

time.sleep(2)

# TODO: Add filtered input config

print("Now starting anticogging calibration")

odrv0.axis0.controller.config.anticogging.end_vel = 0.15
odrv0.axis0.controller.config.anticogging.start_vel = 0.6
odrv0.axis0.controller.config.anticogging.max_torque = 5
odrv0.axis0.controller.config.anticogging.end_gain = 5
odrv0.axis0.controller.config.anticogging.end_tolerance = 0.1

print("Temporarily increasing vel gain and vel integrator gain")

odrv0.axis0.controller.config.vel_integrator_gain = 0.4
odrv0.axis0.controller.config.vel_gain = 1.5

time.sleep(1)

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.start_anticogging_calibration()

print("This will take a while...")

iteration_counter = 0

while odrv0.axis0.controller.config.anticogging.calib_anticogging:
    time.sleep(0.1)

    if iteration_counter % 50 == 0:
        print("Current vel_estimate:", odrv0.encoder_estimator0.vel_estimate)

    iteration_counter += 1

odrv0.axis0.requested_state = AXIS_STATE_IDLE

odrv0.axis0.controller.config.vel_integrator_gain = 0.2
odrv0.axis0.controller.config.vel_gain = 0.5

print("Removing calculated antigocgging bias...")

odrv0.axis0.controller.anticogging_remove_bias()

odrv0.axis0.controller.config.anticogging.pre_calibrated = True

save_config(odrv0)

time.sleep(2)

print("Configuration finished.")

