#! /usr/bin/env python
import argparse
from dronekit import connect
import time

global drone
flag = False

def connectDrone():
    drone = connect('/dev/ttyACM0', baud = 57600, wait_ready = False) 
    #If connnected from telem1:'/dev/ttyTHS1'
    time.sleep(2)



connectDrone()

