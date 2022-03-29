#!/usr/bin/env python3

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import math
import matplotlib.pyplot as plt
import signal,sys,time
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 42068

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock.bind((UDP_IP, UDP_PORT))

terminate = False

def signal_handling(signum,frame):
    global terminate
    terminate = True

signal.signal(signal.SIGINT,signal_handling)

print("Waiting for Hip3 servo to be connected...")
hip_3_servo = odrive.find_any(serial_number=str(hex(56449684681264).split('x')[-1]).upper()) # Uppermost hip (Hip 3)
print("Hip3 connected.")

print("Waiting for Hip2 servo to be connected...")
hip_2_servo = odrive.find_any(serial_number=str(hex(56569943044656).split('x')[-1]).upper()) # Hip 2
print("Hip2 connected.")

print("Waiting for Hip1 servo to be connected...")
hip_1_servo = odrive.find_any(serial_number=str(hex(56612892848688).split('x')[-1]).upper()) # Hip 1
print("Hip1 connected.")

print("Waiting for Knee servo to be connected...")
knee_servo = odrive.find_any(serial_number=str(hex(56557058142768).split('x')[-1]).upper()) # Knee
print("Knee connected.")

hip_3_zero_offset = 0.2002742886543274 # in turns
hip_2_zero_offset = -0.457975834608078 # in turns
hip_1_zero_offset = -0.016931354999542236 # in turns
knee_zero_offset = -0.04928791522979736 # in turns

hip_3_servo.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
hip_2_servo.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
hip_1_servo.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
knee_servo.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

hip_3_servo.axis0.controller.input_pos = hip_3_zero_offset
hip_2_servo.axis0.controller.input_pos = hip_2_zero_offset
hip_1_servo.axis0.controller.input_pos = hip_1_zero_offset
knee_servo.axis0.controller.input_pos = knee_zero_offset

hip_3_lower_limit = -0.48 # in rad
hip_2_lower_limit = -0.8 # in rad
hip_1_lower_limit = -0.4 # in rad
knee_lower_limit = -1.2 # in rad

hip_3_upper_limit = 0.48 # in rad
hip_2_upper_limit = 0.25 # in rad
hip_1_upper_limit = 1.4 # in rad
knee_upper_limit = 1.2 # in rad

gear_ratio = 1/8 # planetary gearbox ratio

# plt.ion()
# plt.show()

data = []

def out_of_bounds(lower, upper, angle):
    return angle < lower or angle > upper

while True:
    if terminate:
        print("CTRL + C detected, deactivating servos and exiting...")
        hip_3_servo.axis0.requested_state = hip_2_servo.axis0.requested_state = hip_1_servo.axis0.requested_state = knee_servo.axis0.requested_state = AXIS_STATE_IDLE
        break

    theta1_raw = hip_3_servo.axis0.pos_vel_mapper.pos_rel
    theta2_raw = hip_2_servo.axis0.pos_vel_mapper.pos_rel
    theta3_raw = hip_1_servo.axis0.pos_vel_mapper.pos_rel
    theta4_raw = knee_servo.axis0.pos_vel_mapper.pos_rel

    theta1dot_raw = hip_3_servo.encoder_estimator0.vel_estimate
    theta2dot_raw = hip_2_servo.encoder_estimator0.vel_estimate
    theta3dot_raw = hip_1_servo.encoder_estimator0.vel_estimate
    theta4dot_raw = knee_servo.encoder_estimator0.vel_estimate

    theta1dot = 2 * math.pi * gear_ratio * theta1dot_raw
    theta2dot = 2 * math.pi * gear_ratio * theta2dot_raw
    theta3dot = 2 * math.pi * gear_ratio * theta3dot_raw
    theta4dot = 2 * math.pi * gear_ratio * theta4dot_raw

    theta1 = 2 * math.pi * gear_ratio * (hip_3_zero_offset - theta1_raw)
    theta2 = 2 * math.pi * gear_ratio * (hip_2_zero_offset - theta2_raw)
    theta3 = 2 * math.pi * gear_ratio * (hip_1_zero_offset - theta3_raw)
    theta4 = 2 * math.pi * gear_ratio * (knee_zero_offset - theta4_raw)

    sock.sendto("{theta1}|{theta2}|{theta3}|{theta4}|0|{theta1dot}|{theta2dot}|{theta3dot}|{theta4dot}|0".format(theta1=theta1, theta2=theta2, theta3=theta3, theta4=theta4, theta1dot=theta1dot, theta2dot=theta2dot, theta3dot=theta3dot, theta4dot=theta4dot).encode(), (UDP_IP, UDP_PORT))
    print("after send")
    # data, addr = sock.recvfrom(4096) # buffer size is 1024 bytes
    # print("received message: %s" % data)


    if out_of_bounds(hip_3_lower_limit, hip_3_upper_limit, theta1) or out_of_bounds(hip_2_lower_limit, hip_2_upper_limit, theta2) or out_of_bounds(hip_1_lower_limit, hip_1_upper_limit, theta3) or out_of_bounds(knee_lower_limit, knee_upper_limit, theta4):
        # print("hip3 before axis idle:", hip_3_servo.axis0.pos_vel_mapper.pos_rel)
        if hip_3_servo.axis0.current_state != AXIS_STATE_IDLE:
            hip_3_servo.axis0.requested_state = AXIS_STATE_IDLE

        if hip_2_servo.axis0.current_state != AXIS_STATE_IDLE:
            hip_2_servo.axis0.requested_state = AXIS_STATE_IDLE

        if hip_1_servo.axis0.current_state != AXIS_STATE_IDLE:
            hip_1_servo.axis0.requested_state = AXIS_STATE_IDLE

        if knee_servo.axis0.current_state != AXIS_STATE_IDLE:
            knee_servo.axis0.requested_state = AXIS_STATE_IDLE

        # hip_3_servo.axis0.requested_state = hip_2_servo.axis0.requested_state = hip_1_servo.axis0.requested_state = knee_servo.axis0.requested_state = AXIS_STATE_IDLE
        # print("hip3 after axis idle:", hip_3_servo.axis0.pos_vel_mapper.pos_rel)

        print("Angles out of safe bounds, deactivating all servos!")

    # print("Hip3:", theta1, ", Hip2:", theta2, ", Hip1:", theta3, ", Knee:", theta4)
    # print("Hip3_raw:", theta1_raw, ", Hip2_raw:", theta2_raw, ", Hip1_raw:", theta3_raw, ", Knee_raw:", theta4_raw)
    # print("Hip3:", math.degrees(theta1), ", Hip2:", math.degrees(theta2), ", Hip1:", math.degrees(theta3), ", Knee:", math.degrees(theta4))

    # data.append(theta1_raw)
    # plt.clf()
    # plt.plot(data)
    # plt.draw()
    # plt.pause(0.01)
    time.sleep(0.001)