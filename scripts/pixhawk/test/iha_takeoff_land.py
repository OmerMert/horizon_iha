#! /usr/bin/env python
import rospy
from dronekit import connect, VehicleMode
import time
altitude = 1 #meters

def connectDrone():
    drone = connect('/dev/ttyACM0', baud = 57600, wait_ready = False) 
    drone.wait_ready('autopilot_version')
    print('Autopilot version: %s'%drone.version)
    time.sleep(2)
    return drone

def arm(drone):
    while drone.is_armable is not True:
        print("drone is not armable")
        time.sleep(2)

    print("drone is armable")
    drone.mode = VehicleMode("GUIDED")
    print("drone mode setted to " + str(drone.mode))
    drone.armed = True

    while drone.armed is not True:
        print("drone is arming..")
        time.sleep(1)

    print("drone armed")


def takeoff(drone, altitude):
    arm(drone)
    drone.simple_takeoff(altitude)

    while drone.location.global_relative_frame.alt < altitude * 0.8:
        print("drone is rising")
        time.sleep(2)

def land(drone):
    drone.mode = VehicleMode("LAND")

    while drone.mode!='LAND':
        time.sleep(1)
        print("Waiting for drone to land")


def main():
    drone = connectDrone()
    takeoff(drone, altitude)
    land(drone)
   

    
# Driver code.
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Closing")
        exit()

