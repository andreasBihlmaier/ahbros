#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
import argparse
import numpy as np
import image_geometry
import cv2
import sys
import numbers
import math
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from ahbros.msg import *
from ahbros.srv import *


def toInt(val):
  if isinstance(val, numbers.Number):
    return int(val)
  elif isinstance(val, Point2D):
    return (int(val.x), int(val.y))
  elif isinstance(val, Point):
    return (int(val.x), int(val.y), int(val.z))
  else:
    return tuple(toInt(fval) for fval in val)


def toFloatTuple(point):
  return (point.x, point.y, point.z)


def toCV(colorRGBA):
  return tuple(int(v * 255) for v in (colorRGBA.b, colorRGBA.g, colorRGBA.r))


def containsNaN(vals):
  for val in vals:
    if math.isnan(val):
      return True
  return False



class ImageProjector(object):
  def __init__(self):
    rospy.loginfo('Waiting for camera info')
    self.camera_info = rospy.wait_for_message('camera_info', CameraInfo)
    print('camera_info:\n%s' % self.camera_info)
    self.geometry_camera = image_geometry.PinholeCameraModel()
    self.geometry_camera.fromCameraInfo(self.camera_info)

    self.image_pub = rospy.Publisher('image_project', Image, queue_size = 1)
    self.image_sub = rospy.Subscriber('image_raw', Image, self.on_image)
    self.points_service = rospy.Service('project_points3d', SetProjectPoints3D, self.on_project_points3d)
    self.points_service = rospy.Service('project_points2d', SetProjectPoints2D, self.on_project_points2d)

    self.last_cv2_image = None
    self.overlay_image = None
    self.overlay_mask = None
    self.points2d = None
    self.points3d = None


  def redraw_overlay(self):
    if self.last_cv2_image is None:
      return

    self.overlay_image = np.zeros(self.last_cv2_image.shape, np.uint8)
    self.overlay_mask = np.zeros(self.last_cv2_image.shape[:2], np.uint8)

    if self.points2d:
      for (point2d_index, point2d) in enumerate(self.points2d.points):
        if len(self.points2d.points) == len(self.points2d.sizes):
          size = self.points2d.sizes[point2d_index]
        else:
          size = self.points2d.sizes[0]
        if len(self.points2d.points) == len(self.points2d.colors):
          color = self.points2d.colors[point2d_index]
        else:
          color = self.points2d.colors[0]
        cv2.circle(self.overlay_image, toInt(point2d), int(size), toCV(color), -1)
        cv2.circle(self.overlay_mask, toInt(point2d), int(size), 255, -1)

    if self.points3d:
      for (point3d_index, point3d) in enumerate(self.points3d.points):
        if len(self.points3d.points) == len(self.points3d.sizes):
          size = self.points3d.sizes[point3d_index]
        else:
          size = self.points3d.sizes[0]
        if len(self.points3d.points) == len(self.points3d.colors):
          color = self.points3d.colors[point3d_index]
        else:
          color = self.points3d.colors[0]
        projected_point = self.geometry_camera.project3dToPixel(toFloatTuple(point3d))
        if containsNaN(projected_point):
          rospy.logerr('Projection of %s contains NaNs' % point3d)
        else:
          cv2.circle(self.overlay_image, toInt(projected_point), int(size), toCV(color), -1)
          cv2.circle(self.overlay_mask, toInt(projected_point), int(size), 255, -1)


  def on_image(self, ros_img):
    bridge = CvBridge()
    try:
      cv2_img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
    except CvBridgeError, e:
      print(e)
    self.last_cv2_image = cv2_img

    if not self.image_pub:
      return

    if not self.overlay_image is None:
      cv2.add(cv2_img, self.overlay_image, cv2_img, self.overlay_mask)

    ros_img_project = bridge.cv2_to_imgmsg(cv2_img, "bgr8")
    self.image_pub.publish(ros_img_project)


  def validate_points(self, points_msg):
    if len(points_msg.points.sizes) != 1 and len(points_msg.points.sizes) != len(points_msg.points.points):
      rospy.logerr('SetProjectPoints*.points.sizes must contain either one or len(points) values')
      return False

    if len(points_msg.points.colors) != 1 and len(points_msg.points.colors) != len(points_msg.points.points):
      rospy.logerr('SetProjectPoints*.points.colors must contain either one or len(points) values')
      return False

    return True


  def on_project_points3d(self, points_msg):
    print('Got 3D points:\n%s' % points_msg.points)

    if not self.validate_points(points_msg):
      return SetProjectPoints3DResponse(False)

    self.points3d = points_msg.points
    self.redraw_overlay()
    return SetProjectPoints3DResponse(True)


  def on_project_points2d(self, points_msg):
    print('Got 2D points:\n%s' % points_msg.points)

    if not self.validate_points(points_msg):
      return SetProjectPoints2DResponse(False)

    self.points2d = points_msg.points
    self.redraw_overlay()
    return SetProjectPoints2DResponse(True)



def main(args):
  parser = argparse.ArgumentParser()
  #parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are republished (default: 10 Hz)')
  #parser.add_argument('robots_count', type=int, help='Number of merged robots')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('project_to_image', anonymous=True)
  image_projector = ImageProjector()

  rospy.loginfo('Spinning')
  rospy.spin()


if __name__ == '__main__':
  main(sys.argv)
