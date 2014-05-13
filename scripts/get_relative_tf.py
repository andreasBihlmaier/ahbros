#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import tf
import numpy
from tf.transformations import *



def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('from_tf', help='Base Frame')
  parser.add_argument('to_tf', help='Target Frame')
  args = parser.parse_args(rospy.myargv()[1:])
  from_tf, to_tf = args.from_tf, args.to_tf

  rospy.init_node('get_relative_tf', anonymous=True)
  tf_listener = tf.TransformListener()

  tf_listener.waitForTransform(to_tf, from_tf, rospy.Time(), rospy.Duration(4.0))
  position, quaternion = tf_listener.lookupTransform(to_tf, from_tf, rospy.Time())
  print('Translation: %s' % str(position))
  print('Quaternion: %s' % str(quaternion))
  matrix = numpy.dot(translation_matrix(position), quaternion_matrix(quaternion))
  print('Matrix:\n%s' % matrix)


if __name__ == '__main__':
  main(sys.argv)
