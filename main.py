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
pi.set_mode(SERVO, pigpio.OUTPUT)

pi.set_PWM_frequency(SERVO, 50)
pi.set_PWM_range(SERVO, 2000) # 1,000,000 / 50 = 20,000us for 100% duty cycle
pi.set_servo_pulsewidth(SERVO, 500) # Minimum throttle.

time.sleep(10)

###
pi.stop() # Disconnect from local Raspberry Pi.