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

pi.set_servo_pulsewidth(SERVO, 500) # Minimum throttle.

time.sleep(10)

###
pi.stop() # Disconnect from local Raspberry Pi.