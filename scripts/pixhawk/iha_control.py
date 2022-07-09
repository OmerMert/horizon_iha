#! /usr/bin/env python
import argparse
import rospy
from std_msgs.msg import Bool
from dronekit import Command, connect, VehicleMode
import time
from pymavlink import mavutil

global drone
flag = False

def connectDrone():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate = 57600

    drone = connect(connection_string, baud = baud_rate, wait_ready = True)
    time.sleep(2)

def takeoff(altitude):

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
    drone.simple_takeoff(altitude)

    while drone.location.global_relative_frame.alt < altitude * 0.9:
        print("drone is rising")
        time.sleep(2)


def add_mission():
    
    global command 
    command = drone.commands
    
    command.clear()
    time.sleep(1)

    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 7))
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,  -35.36304146, 149.16525017, 7))

    command.upload()


def detection_callback(area_detected):
    global flag 
    if (area_detected.data) and (flag is False):
        print('Captured the Location of Area!')
        go_to_area(drone.location.global_relative_frame.lat,drone.location.global_relative_frame.lon)
        flag = True

def go_to_area(area_x, area_y):
    
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, area_x, area_y, 7))
    command.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, area_x, area_y, 2))
    
    command.upload()

def main():
    connectDrone()
    takeoff(2)
    drone.mode = VehicleMode("AUTO")
    add_mission()
    
    rospy.init_node("uav_controller", anonymous=True)
    
    detec_sub = rospy.Subscriber("area_detection", Bool, detection_callback)

    
# Driver code.
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Closing")
        exit()
