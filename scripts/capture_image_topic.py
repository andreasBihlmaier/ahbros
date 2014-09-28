#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-t', '--topic', default='image_raw', help='Image topic')
  parser.add_argument('-o', '--output_file', default='/tmp/image.png', help='Image destination file')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('capture_image_topic', anonymous=True)

  rospy.loginfo('Waiting for image msg on %s' % args.topic)
  ros_img = rospy.wait_for_message(args.topic, Image)

  bridge = CvBridge()
  try:
    cv2_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
  except CvBridgeError, e:
    print e

  cv2.imwrite(args.output_file, cv2_img)


if __name__ == '__main__':
  main()
