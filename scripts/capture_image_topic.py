#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

all_captured = False

def on_image(ros_img):
  bridge = CvBridge()
  try:
    cv_img = bridge.imgmsg_to_cv(ros_img, "bgr8")
    cv2_img = np.array(cv_img, dtype=np.uint8)
  except CvBridgeError, e:
    print e

  cv2.imwrite('/tmp/test.jpg', cv2_img)
  global all_captured
  all_captured = True

def main(args):
  rospy.init_node('capture_image_topic', anonymous=True)

  image_subscriber = rospy.Subscriber('image_raw', Image, on_image)

  rospy.loginfo('Spinning')
  global all_captured
  while not all_captured:
    rospy.sleep(rospy.Duration(0, 10 * 1000))

if __name__ == '__main__':
  main(sys.argv)
