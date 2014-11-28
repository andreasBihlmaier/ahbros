#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse
import tf
import numpy as np
from tf.transformations import *
from geometry_msgs.msg import Pose



def homogeneous2translation_quaternion(homogeneous):
  """
  Translation: [x, y, z]
  Quaternion: [x, y, z, w]
  """
  translation = translation_from_matrix(homogeneous)
  quaternion = quaternion_from_matrix(homogeneous)
  return translation, quaternion


def homogeneous2pose_msg(homogeneous):
  pose = Pose()
  translation, quaternion = homogeneous2translation_quaternion(homogeneous)
  pose.position.x = translation[0]
  pose.position.y = translation[1]
  pose.position.z = translation[2]
  pose.orientation.x = quaternion[0]
  pose.orientation.y = quaternion[1]
  pose.orientation.z = quaternion[2]
  pose.orientation.w = quaternion[3]
  return pose



def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('from_tf', help='Base Frame')
  parser.add_argument('to_tf', help='Target Frame')
  args = parser.parse_args(rospy.myargv()[1:])
  from_tf, to_tf = args.from_tf, args.to_tf

  #print('Consider using: rosrun tf2 tf2_echo %s %s' % (from_tf, to_tf))

  rospy.init_node('get_relative_tf', anonymous=True)
  tf_listener = tf.TransformListener()

  tf_listener.waitForTransform(from_tf, to_tf, rospy.Time(), rospy.Duration(4.0))
  position, quaternion = tf_listener.lookupTransform(from_tf, to_tf, rospy.Time())
  print('Translation: %s' % str(position))
  print('Quaternion: %s' % str(quaternion))
  matrix = np.dot(translation_matrix(position), quaternion_matrix(quaternion))
  print('Matrix:\n%s' % matrix)
  poseMsg = homogeneous2pose_msg(matrix)
  print('geometry_msgs/Pose:\n%s' % poseMsg)
  print('As static publisher: rosrun tf static_transform_publisher  %s  %s  %s %s 100' % (' '.join([str(p) for p in position]), ' '.join([str(q) for q in quaternion]), from_tf, to_tf))


if __name__ == '__main__':
  main(sys.argv)
