#! /usr/bin/env python
import rospy
from dronekit import connect
import time

global drone
flag = False

def connectDrone():
    drone = connect('/dev/ttyACM0', baud = 57600, wait_ready = False)
    #If connnected from telem1:'/dev/ttyTHS1'
    drone.wait_ready('autopilot_version')
    print('Autopilot version: %s'%drone.version)
    time.sleep(2)
    return drone

def main():
    connectDrone()
    
    rospy.init_node("uav_connect", anonymous=True)

    
# Driver code.
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Closing")
        exit()
