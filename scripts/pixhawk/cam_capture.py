#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import cv2
import color_detection
from std_msgs.msg import Bool


bridge = CvBridge()

def image_callback(ros_image):
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  area_detected = color_detection.detect_center(cv_image)
  pub = rospy.Publisher("area_detection", Bool, queue_size= 10)
  pub.publish(area_detected)
  
  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  image_sub = rospy.Subscriber('dev/video0',Image, image_callback)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)