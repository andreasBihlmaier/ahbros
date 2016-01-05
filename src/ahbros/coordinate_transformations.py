from math import acos, sqrt
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


def homogeneous2quaternion(homogeneous):
  """
  Quaternion: [x, y, z, w]
  """
  quaternion = quaternion_from_matrix(homogeneous)
  return quaternion


def homogeneous2rpy(homogeneous):
  """
  RPY: [sx, sy, sz]
  """
  rpy = euler_from_matrix(homogeneous)
  return rpy


def homogeneous2axis_angle(homogeneous):
  """
  Axis-angle: [ax, ay, az]
  """
  qx, qy, qz, qw = homogeneous2quaternion(homogeneous)
  angle = 2.0 * acos(qw)
  norm = sqrt(1 - qw*qw)
  ax = (qx / norm) * angle
  ay = (qy / norm) * angle
  az = (qz / norm) * angle
  return (ax, ay, az)


