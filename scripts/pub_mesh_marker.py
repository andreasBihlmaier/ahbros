#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import argparse
import thread

import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray



class MarkerPublisher(object):
  def __init__(self, marker_topic = '/visualization_marker', frequency = 1):
    self.marker_topic = marker_topic
    self.frequency = frequency
    rospy.loginfo('Will publish markers on %s with %d Hz' % (self.marker_topic, self.frequency))
    self.marker_msgs = []
    self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=100)
    self.marker_thread = None

  def publish_markers(self):
    rate = rospy.Rate(self.frequency)
    while not rospy.is_shutdown():
      for marker_msg in self.marker_msgs: 
        marker_msg.header.stamp = rospy.get_rostime()
        self.marker_pub.publish(marker_msg)
        rate.sleep()

  def pub_marker_sync(self, marker_msg):
    cnt = 1
    while not rospy.is_shutdown() and self.marker_pub.get_num_connections() == 0:
      if cnt % 10 == 0:
        rospy.loginfo('Waiting for subscription to /visualization_marker')
      rospy.sleep(0.1)
      self.marker_pub.publish(marker_msg)
      cnt += 1
    self.marker_pub.publish(marker_msg)
    self.marker_msgs.append(marker_msg)

  def add_marker(self, mesh_frame, marker_name, mesh_file):
    marker_msg = Marker()
    marker_msg.frame_locked = True
    marker_msg.id = 0
    marker_msg.action = Marker.ADD
    marker_msg.mesh_use_embedded_materials = False
    marker_msg.color.a = 1.0
    marker_msg.color.r = 0.6
    marker_msg.color.g = 0.6
    marker_msg.color.b = 0.6
    marker_msg.header.stamp = rospy.get_rostime()
    #marker_msg.lifetime =
    #marker_msg.pose =
    marker_msg.type = Marker.MESH_RESOURCE
    marker_msg.header.frame_id = mesh_frame
    marker_msg.ns = marker_name
    marker_msg.mesh_resource = 'file://%s' % (os.path.abspath(mesh_file))
    scale = 1.0
    marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale, scale, scale
    marker_msg.pose.position.x, marker_msg.pose.position.y, marker_msg.pose.position.z = 0, 0, 0
    marker_msg.pose.orientation.x, marker_msg.pose.orientation.y, marker_msg.pose.orientation.z, marker_msg.pose.orientation.w = 0, 0, 0, 1
    self.pub_marker_sync(marker_msg)

    if not self.marker_thread:
      self.marker_thread = thread.start_new_thread(MarkerPublisher.publish_markers, (self,))




def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--frequency', nargs=1, type=int, default=1, help='Publish frequency')
  parser.add_argument('-t', '--topic', nargs=1, type=str, default='/visualization_marker', help='Marker topic')
  parser.add_argument('mesh_frame', help='Mesh frame')
  parser.add_argument('marker_name', help='Marker name (=namespace)')
  parser.add_argument('mesh_file', help='Mesh file')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('pub_mesh_marker', anonymous = True)

  marker_pub = MarkerPublisher(args.topic if type(args.topic) == str else args.topic[0], args.frequency)
  marker_pub.add_marker(args.mesh_frame, args.marker_name, args.mesh_file)

  rospy.loginfo('Spinning')
  rospy.spin()


if __name__ == '__main__':
  main()
