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
  parser.add_argument('-f', '--frequency', nargs=1, type=int, default=1, help='Publish frequency')
  parser.add_argument('-e', '--encoding', nargs=1, type=str, help='Force encoding of published images (e.g. mono8 or bgr8)')
  parser.add_argument('images', metavar='image', nargs='+', help='Images to publish')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('publish_image_from_file', anonymous=True)

  bridge = CvBridge()
  image_pub = rospy.Publisher('image_raw', Image, queue_size=1, latch=True)
  rate = rospy.Rate(args.frequency)

  while not rospy.is_shutdown():
    for image_file in args.images:
      cv_img = cv2.imread(image_file)
      if cv_img.shape[2] == 3:
        cv_img_encoding = 'bgr8'
      else:
        cv_img_encoding = 'mono8'

      if args.encoding:
        encoding = args.encoding[0]
        if cv_img_encoding == 'bgr8' and encoding == 'mono8':
          cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        elif cv_img_encoding == 'mono8' and encoding == 'bgr8':
          cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
      else:
        encoding = cv_img_encoding

      try:
        img_msg = bridge.cv2_to_imgmsg(cv_img, encoding)
        img_msg.header.stamp = rospy.get_rostime()
        image_pub.publish(img_msg)
      except CvBridgeError, e:
        print e

      rate.sleep()



if __name__ == '__main__':
  main()
