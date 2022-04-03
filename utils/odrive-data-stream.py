#!/usr/bin/env python3

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import math
import matplotlib.pyplot as plt
import signal,sys,time
import socket

controller_ip = "127.0.0.1"
controller_port = 4200

rviz_ip = "192.168.122.168"
rviz_port = 42070

controller_socket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Bind to temporary random port, doesn't matter which one
# controller_socket.bind((controller_ip, 42068))

rviz_socket = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

# Bind to temporary random port, doesn't matter which one
# rviz_socket.bind(("127.0.0.1", 42067))

terminate = False

def signal_handling(signum,frame):
    global terminate
    terminate = True

signal.signal(signal.SIGINT,signal_handling)

# print("Waiting for Hip3 servo to be connected...")
# hip_3_servo = odrive.find_any(serial_number=str(hex(56449684681264).split('x')[-1]).upper()) # Uppermost hip (Hip 3)
# print("Hip3 connected.")

print("Waiting for Hip2 servo to be connected...")
hip_2_servo = odrive.find_any(serial_number=str(hex(56569943044656).split('x')[-1]).upper()) # Hip 2
print("Hip2 connected.")

print("Waiting for Hip1 servo to be connected...")
hip_1_servo = odrive.find_any(serial_number=str(hex(56612892848688).split('x')[-1]).upper()) # Hip 1
print("Hip1 connected.")

print("Waiting for Knee servo to be connected...")
knee_servo = odrive.find_any(serial_number=str(hex(56557058142768).split('x')[-1]).upper()) # Knee
print("Knee connected.")

hip_3_zero_offset = 0 # in turns
hip_2_zero_offset = 0.016779117286205292 # in turns
hip_1_zero_offset = -0.0005703568458557129 # in turns
knee_zero_offset =  0.015657007694244385 # in turns

arm_servos = False

if arm_servos:
    # hip_3_servo.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    hip_2_servo.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    hip_1_servo.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    knee_servo.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# hip_3_servo.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
hip_2_servo.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
hip_1_servo.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
knee_servo.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

# hip_3_servo.axis0.controller.input_torque = 0
hip_2_servo.axis0.controller.input_torque = 0
hip_1_servo.axis0.controller.input_torque = 0
knee_servo.axis0.controller.input_torque = 0

hip_3_lower_limit = -0.5 # in rad
hip_2_lower_limit = -0.25 # in rad
hip_1_lower_limit = -0.4 # in rad
knee_lower_limit = -1.2 # in rad

hip_3_upper_limit = 0.3 # in rad
hip_2_upper_limit = 0.2 # in rad
hip_1_upper_limit = 1.1 # in rad
knee_upper_limit = 1.2 # in rad


# Sign adjustments for angle and angular velocity readings
hip_3_sign = -1
hip_2_sign = 1
hip_1_sign = 1
knee_sign = -1

# Sign adjustments for torque commands
hip_3_torque_sign = 1
hip_2_torque_sign = -1
hip_1_torque_sign = -1
knee_torque_sign = 1

gear_ratio = 1 # planetary gearbox ratio
print("Gear ratio is", gear_ratio)

# plt.ion()
# plt.show()

message = "{theta1}|{theta2}|{theta3}|{theta4}|0|{theta1dot}|{theta2dot}|{theta3dot}|{theta4dot}|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0".format(theta1=0, theta2=0, theta3=0, theta4=0, theta1dot=0, theta2dot=0, theta3dot=0, theta4dot=0)
controller_socket.sendto(message.encode(), (controller_ip, controller_port))

data = []

def out_of_bounds(lower, upper, angle):
    return angle < lower or angle > upper

while True:
    # start = time.time()

    if terminate:
        print("CTRL + C detected, deactivating servos and exiting...")
        hip_2_servo.axis0.requested_state = hip_1_servo.axis0.requested_state = knee_servo.axis0.requested_state = AXIS_STATE_IDLE
        break

    # theta1_raw = hip_3_servo.axis0.pos_vel_mapper.pos_rel
    theta1_raw = 0
    theta2_raw = hip_2_servo.axis0.pos_vel_mapper.pos_rel
    theta3_raw = hip_1_servo.axis0.pos_vel_mapper.pos_rel
    theta4_raw = knee_servo.axis0.pos_vel_mapper.pos_rel

    # theta1dot_raw = hip_3_servo.encoder_estimator0.vel_estimate
    theta1dot_raw = 0
    theta2dot_raw = hip_2_servo.encoder_estimator0.vel_estimate
    theta3dot_raw = hip_1_servo.encoder_estimator0.vel_estimate
    theta4dot_raw = knee_servo.encoder_estimator0.vel_estimate

    # theta1_raw = theta2_raw = theta3_raw = theta4_raw = 0.1
    # theta1dot_raw = theta2dot_raw = theta3dot_raw = theta4dot_raw = -0.05

    theta1dot = 2 * math.pi * gear_ratio * theta1dot_raw * hip_3_sign
    theta2dot = 2 * math.pi * gear_ratio * theta2dot_raw * hip_2_sign
    theta3dot = 2 * math.pi * gear_ratio * theta3dot_raw * hip_1_sign
    theta4dot = 2 * math.pi * gear_ratio * theta4dot_raw * knee_sign

    theta1dot = 0

    theta1 = 2 * math.pi * gear_ratio * (hip_3_zero_offset - theta1_raw) * hip_3_sign
    theta2 = 2 * math.pi * gear_ratio * (hip_2_zero_offset - theta2_raw) * hip_2_sign
    theta3 = 2 * math.pi * gear_ratio * (hip_1_zero_offset - theta3_raw) * hip_1_sign
    theta4 = 2 * math.pi * gear_ratio * (knee_zero_offset - theta4_raw) * knee_sign
    
    theta1 = 0

    # Compensate for linkage
    theta4 -= theta3
    theta4dot -= theta3dot

    if out_of_bounds(hip_3_lower_limit, hip_3_upper_limit, theta1) or out_of_bounds(hip_2_lower_limit, hip_2_upper_limit, theta2) or out_of_bounds(hip_1_lower_limit, hip_1_upper_limit, theta3) or out_of_bounds(knee_lower_limit, knee_upper_limit, theta4):
        # hip_3_servo.axis0.controller.input_torque = 0
        hip_2_servo.axis0.controller.input_torque = 0
        hip_1_servo.axis0.controller.input_torque = 0
        knee_servo.axis0.controller.input_torque = 0
        
        # if hip_3_servo.axis0.current_state != AXIS_STATE_IDLE:
        #     hip_3_servo.axis0.requested_state = AXIS_STATE_IDLE

        if hip_2_servo.axis0.current_state != AXIS_STATE_IDLE:
            hip_2_servo.axis0.requested_state = AXIS_STATE_IDLE

        if hip_1_servo.axis0.current_state != AXIS_STATE_IDLE:
            hip_1_servo.axis0.requested_state = AXIS_STATE_IDLE

        if knee_servo.axis0.current_state != AXIS_STATE_IDLE:
            knee_servo.axis0.requested_state = AXIS_STATE_IDLE
        
        print("Angles out of safe bounds, deactivating all servos!")
    
    message = "{theta1}|{theta2}|{theta3}|{theta4}|0|{theta1dot}|{theta2dot}|{theta3dot}|{theta4dot}|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0".format(theta1=theta1, theta2=theta2, theta3=theta3, theta4=theta4, theta1dot=theta1dot, theta2dot=theta2dot, theta3dot=theta3dot, theta4dot=theta4dot)
    controller_socket.sendto(message.encode(), (controller_ip, controller_port))
    rviz_socket.sendto(message.encode(), (rviz_ip, rviz_port))
    
    data, addr = controller_socket.recvfrom(4096) # buffer size is 4096 bytes
    # print("received message: %s" % data)

    tau_raw = [float(x) for x in data.decode().split("|")[:-1]]

    tau_1 = tau_raw[0] * gear_ratio * hip_3_torque_sign
    tau_2 = tau_raw[1] * gear_ratio * hip_2_torque_sign
    tau_3 = tau_raw[2] * gear_ratio * hip_1_torque_sign
    tau_4 = tau_raw[3] * gear_ratio * knee_torque_sign

    # tau_1 = 0
    # tau_2 = 0 * hip_2_torque_sign
    # tau_3 = 0 * hip_1_torque_sign
    # tau_4 = 2 * knee_torque_sign
    
    # print("tau_1:", tau_1, "tau_2:", tau_2, "tau_3:", tau_3, "tau_4:", tau_4)

    # hip_3_servo.axis0.controller.input_torque = tau_1
    hip_2_servo.axis0.controller.input_torque = tau_2
    hip_1_servo.axis0.controller.input_torque = tau_3
    knee_servo.axis0.controller.input_torque = tau_4

    # hip_3_servo.axis0.controller.input_torque = 0.5 * hip_3_torque_sign
    # hip_2_servo.axis0.controller.input_torque = 0.5 * hip_2_torque_sign
    # hip_1_servo.axis0.controller.input_torque = 0.5 * hip_1_torque_sign
    # knee_servo.axis0.controller.input_torque = 0.5 * knee_torque_sign

    print("Hip3:", theta1, ", Hip2:", theta2, ", Hip1:", theta3, ", Knee:", theta4)
    print("Hip3_raw:", theta1_raw, ", Hip2_raw:", theta2_raw, ", Hip1_raw:", theta3_raw, ", Knee_raw:", theta4_raw)
    # print("Hip3:", math.degrees(theta1), ", Hip2:", math.degrees(theta2), ", Hip1:", math.degrees(theta3), ", Knee:", math.degrees(theta4))

    # data.append(theta1_raw)
    # plt.clf()
    # plt.plot(data)
    # plt.draw()
    # plt.pause(0.01)

    # end = time.time()
    # print(end - start)
    # time.sleep(0.005)