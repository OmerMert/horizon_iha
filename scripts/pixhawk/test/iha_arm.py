#! /usr/bin/env python
import rospy
from dronekit import connect, VehicleMode
import time

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

def main():

    drone = connectDrone()
    arm(drone)
    
    rospy.init_node("uav_controller", anonymous=True)

    
# Driver code.
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Closing")
        exit()

