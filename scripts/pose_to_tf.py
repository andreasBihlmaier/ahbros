#!/usr/bin/env python

import roslib
import rospy
import argparse
import tf
import sys
import numpy as np
from tf.transformations import *
from ahbros.coordinate_transformations import *
from geometry_msgs.msg import PoseStamped



class PoseToTfBroadcaster(object):
  def __init__(self, child_frame_id):
    self.tf_broadcaster = tf.TransformBroadcaster()
    self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.on_pose, queue_size = 1)
    self.child_frame_id = child_frame_id

  def on_pose(self, pose_msg):
    print(pose_msg)
    self.tf_broadcaster.sendTransform((pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z), 
                                      (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w),
                                      pose_msg.header.stamp,
                                      self.child_frame_id,
                                      pose_msg.header.frame_id)



def main(args):
  parser = argparse.ArgumentParser()
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('pose_to_tf', anonymous=True)

  child_frame_id = rospy.get_param("child_frame_id", "pose_to_tf_frame")
  pose_to_tf_broadcaster = PoseToTfBroadcaster(child_frame_id)
  
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
