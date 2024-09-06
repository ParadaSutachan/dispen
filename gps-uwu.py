# -- coding: utf-8 --
#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

GPIO.setup(21, GPIO.OUT)
rojo = GPIO.PWM(21, 50)
rojo.start(0)    

while True:
    for i in range(0, 101, 5):
        rojo.ChangeDutyCycle(i)
        print(i)
        time.sleep(3)           

    print("Ciclo completo")
    pwm.stop()  
    GPIO.cleanup()
