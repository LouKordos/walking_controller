#!/usr/bin/env python3

import odrive
import time
import math
import matplotlib.pyplot as plt

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

hip_3_zero_offset = 0.1792331486940384 # in turns
hip_2_zero_offset = 0.13275079429149628 # in turns
hip_1_zero_offset = 0.9866976141929626 # in turns
knee_zero_offset = 0.26936811208724976 # in turns

hip_3_lower_limit = 0 # in rad
hip_2_lower_limit = 0 # in rad
hip_1_lower_limit = 0 # in rad
knee_lower_limit = 0 # in rad

hip_3_upper_limit = 0 # in rad
hip_2_upper_limit = 0 # in rad
hip_1_upper_limit = 0 # in rad
knee_upper_limit = 0 # in rad

gear_ratio = 1/8 # planetary gearbox ratio

plt.ion()
plt.show()

data = []

while True:
    theta1_raw = hip_3_servo.axis0.pos_vel_mapper.pos_rel
    theta2_raw = hip_2_servo.axis0.pos_vel_mapper.pos_rel
    theta3_raw = hip_1_servo.axis0.pos_vel_mapper.pos_rel
    theta4_raw = knee_servo.axis0.pos_vel_mapper.pos_rel

    theta1dot_raw = hip_3_servo.encoder_estimator0.vel_estimate
    theta2dot_raw = hip_2_servo.encoder_estimator0.vel_estimate
    theta3dot_raw = hip_1_servo.encoder_estimator0.vel_estimate
    theta4dot_raw = knee_servo.encoder_estimator0.vel_estimate

    theta1 = 2 * math.pi * gear_ratio * (hip_3_zero_offset - theta1_raw)
    theta2 = 2 * math.pi * gear_ratio * (hip_2_zero_offset - theta2_raw)
    theta3 = 2 * math.pi * gear_ratio * (hip_1_zero_offset - theta3_raw)
    theta4 = 2 * math.pi * gear_ratio * (knee_zero_offset - theta4_raw)

    print("Hip3:", theta1, ", Hip2:", theta2, ", Hip1:", theta3, ", Knee:", theta4)
    # data.append(gear_ratio * 1 * (hip_1_zero_offset - hip_1_servo.encoder_estimator0.pos_estimate))
    data.append(theta4)
    plt.clf()
    plt.plot(data)
    plt.draw()
    plt.pause(0.01)
    time.sleep(0.01)