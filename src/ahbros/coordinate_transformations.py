from math import acos, sqrt, radians, degrees, pi
from tf.transformations import *
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, Transform, TransformStamped


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


def pose_msg2homogeneous(pose):
  trans = translation_matrix((pose.position.x, pose.position.y, pose.position.z))
  rot = quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
  return concatenate_matrices(trans, rot)


def homogeneous2transform_msg(homogeneous):
  transform = Transform()
  translation, quaternion = homogeneous2translation_quaternion(homogeneous)
  transform.translation.x = translation[0]
  transform.translation.y = translation[1]
  transform.translation.z = translation[2]
  transform.rotation.x = quaternion[0]
  transform.rotation.y = quaternion[1]
  transform.rotation.z = quaternion[2]
  transform.rotation.w = quaternion[3]
  return transform


def transform_msg2homogeneous(transform):
  trans = translation_matrix((transform.translation.x, transform.translation.y, transform.translation.z))
  rot = quaternion_matrix((transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w))
  return concatenate_matrices(trans, rot)


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


def rpy2homogeneous(rpy):
  """
  RPY (aka EulerZYX): [sx, sy, sz]
  """
  homogeneous = euler_matrix(rpy[0], rpy[1], rpy[2], 'rzyx')
  return homogeneous


def rpy2quaternion(rpy):
  """
  RPY (aka EulerZYX): [sx, sy, sz]
  """
  return quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'rzyx')


def euler2quaternion(rpy):
  """
  RPY: [sx, sy, sz]
  """
  return quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'rxyz')


def quaternion_vector2quaternion_ros(quaternion_vector):
  """
  Quaternion: [x, y, z, w]
  """
  quaternion_ros = Quaternion()
  quaternion_ros.x, quaternion_ros.y, quaternion_ros.z, quaternion_ros.w = quaternion_vector
  return quaternion_ros
