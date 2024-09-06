#!/usr/bin/env python

# esc_start.py
# 2015-04-14
# Public Domain
#
# Sends the servo pulses needed to initialise some ESCs
#
# Requires the pigpio daemon to be running
#
# sudo pigpiod

import time

import pigpio

SERVO = 21

pi = pigpio.pi() # Connect to local Pi.

pi.set_servo_pulsewidth(SERVO, 2000) # Minimum throttle.

input("Press enter . ")

time.sleep(2)

input("Press enter")
pi.set_servo_pulsewidth(SERVO, 500) # Maximum throttle.

time.sleep(5)

pi.set_servo_pulsewidth(SERVO, 1250) # Slightly open throttle.

time.sleep(5)

pi.set_servo_pulsewidth(SERVO, 0) # Stop servo pulses.

time.sleep(3)

pi.stop() # Disconnect from local Raspberry Pi.