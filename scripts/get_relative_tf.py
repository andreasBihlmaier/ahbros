#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import tf



def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('from_tf', help='Base Frame')
  parser.add_argument('to_tf', help='Target Frame')
  args = parser.parse_args()
  from_tf, to_tf = args.from_tf, args.to_tf

  rospy.init_node('get_relative_tf', anonymous=True)
  tf_listener = tf.TransformListener()
  rospy.sleep(rospy.Duration(0, 500 * 1000))


  all_frames_available = False
  while not all_frames_available:
    all_frames_available = True
    for frame in from_tf, to_tf:
      if not tf_listener.frameExists(frame):
        print('Frame %s does not exist' % frame)
        all_frames_available = False
    rospy.sleep(rospy.Duration(0, 100 * 1000))

  common_time = tf_listener.getLatestCommonTime(from_tf, to_tf)
  position, quaternion = tf_listener.lookupTransform(from_tf, to_tf, common_time)
  print position, quaternion


if __name__ == '__main__':
  main(sys.argv)
