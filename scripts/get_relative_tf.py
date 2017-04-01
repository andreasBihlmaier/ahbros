#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import tf2_ros
import numpy as np
from tf.transformations import *
from math import acos, sqrt
from ahbros.coordinate_transformations import *



def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('from_tf', help='Base Frame')
  parser.add_argument('to_tf', help='Target Frame')
  args = parser.parse_args(rospy.myargv()[1:])
  from_tf, to_tf = args.from_tf, args.to_tf

  #print('Consider using: rosrun tf2 tf2_echo %s %s' % (from_tf, to_tf))

  rospy.init_node('get_relative_tf', anonymous=True)
  tf_buffer = tf2_ros.Buffer()
  tf_listener = tf2_ros.TransformListener(tf_buffer)

  transform_stamped = tf_buffer.lookup_transform(from_tf, to_tf, rospy.Time(), rospy.Duration(4.0))
  print('geometry_msgs/Transform:\n%s' % transform_stamped.transform)
  homogeneous = transform_msg2homogeneous(transform_stamped.transform)
  print('Homogeneous:\n%s' % homogeneous)
  position, quaternion = homogeneous2translation_quaternion(homogeneous)
  print('Translation: %s' % str(position))
  print('Quaternion: %s' % str(quaternion))
  pose_msg = homogeneous2pose_msg(homogeneous)
  print('geometry_msgs/Pose:\n%s' % pose_msg)
  rpy = homogeneous2rpy(homogeneous)
  print('RPY: %s' % str(rpy))
  axis_angle = homogeneous2axis_angle(homogeneous)
  print('Axis Angle: %s' % str(axis_angle))
  print('As static publisher: rosrun tf static_transform_publisher  %s  %s  %s %s 100' % (' '.join([str(p) for p in position]), ' '.join([str(q) for q in quaternion]), from_tf, to_tf))


if __name__ == '__main__':
  main(sys.argv)
