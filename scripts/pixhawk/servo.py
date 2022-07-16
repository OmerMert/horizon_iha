#! /usr/bin/env python3
#Pin Connection PCA9685 with Jetson Nano
#VCC:3.3V SDA:3 SCL:5 GND:GND(6)

from adafruit_servokit import ServoKit
import busio
import board
import time

i2c_bus0 = busio.I2C(board.SCL, board.SDA)
kit = ServoKit(channels=16,i2c=i2c_bus0)
             

kit.servo[0].angle = 180
time.sleep(5)
kit.servo[0].angle = 0
time.sleep(5)
kit.servo[0].angle = 180
time.sleep(5)
kit.servo[0].angle = 0