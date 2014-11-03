#!/usr/bin/env python

from __future__ import print_function

import copy
import roslib
import rospy
import sys
import argparse
import xml.etree.ElementTree as ET

from sensor_msgs.msg import JointState



class Robot(object):
  def __init__(self, id):
    self.id = id
    self.source_joint_names = []
    self.target_joint_names = []
    self.urdf = rospy.get_param('robot_description%d' % self.id, None)
    if not self.urdf:
      rospy.logerr('No URDF description for robot %d' % self.id)
      return
    self.name = ET.fromstring(self.urdf).attrib['name']
    rospy.loginfo('id=%d name=%s' % (self.id, self.name))
    self.joint_sub = rospy.Subscriber('joint_states%d' % self.id, JointState, self.on_joint)
    self.target_joint_pub = rospy.Publisher('joint_states', JointState)


  def on_joint(self, msg):
    if not self.source_joint_names:
      self.source_joint_names = msg.name
      rospy.loginfo('%d: source_joint_names: %s' % (self.id, self.source_joint_names))
      # TODO check against self.urdf
    if msg.name != self.source_joint_names:
      rospy.logerr('joint names of robot %d changed (old=%s new=%s), this is not supported. Quitting.' % (self.id, self.source_joint_names, msg.name))
      sys.exit(1)

    if self.target_joint_names:
      target_msg = copy.deepcopy(msg)
      target_msg.name = self.target_joint_names
      self.target_joint_pub.publish(target_msg)


  def get_target_joint_names(self, target_urdf):
    while not self.source_joint_names:
      rospy.loginfo('%d: Waiting to receive source_joint_names' % self.id)
      rospy.sleep(1)
    root = ET.fromstring(target_urdf)
    all_target_joint_names = []
    for joint in root.iter('joint'):
      all_target_joint_names.append(joint.attrib['name'])
    rospy.logdebug('%d: all_target_joint_names: %s' % (self.id, all_target_joint_names))
    target_joint_names = []
    for src_name in self.source_joint_names:
      target_names = [joint_name for joint_name in all_target_joint_names if joint_name.endswith(self.name + '__' + src_name)]
      if len(target_names) != 1:
        rospy.logerr('%d: source joint name %s not contained or unique within merged robot: %s. Quitting.' % (self.id, src_name, target_names))
        sys.exit(2)
      target_joint_names.append(target_names[0])
    self.target_joint_names = target_joint_names
    rospy.loginfo('%d: target_joint_names: %s' % (self.id, self.target_joint_names))
    rename_dict = {}
    for (joint_idx, src_joint_name) in enumerate(self.source_joint_names):
      rename_dict[src_joint_name] = self.target_joint_names[joint_idx]
    rospy.loginfo('%d: rename_dict: %s' % (self.id, rename_dict))
    # append dictionary instead of replacing it
    param_dict = rospy.get_param('single_to_composite_joints', {})
    param_dict.update(rename_dict)
    rospy.set_param('single_to_composite_joints', param_dict)




def main(args):
  parser = argparse.ArgumentParser()
  #parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are republished (default: 10 Hz)')
  parser.add_argument('robots_count', type=int, help='Number of merged robots')
  args = parser.parse_args(rospy.myargv()[1:])
  robots_count = args.robots_count

  rospy.init_node('joint_topic_merger')

  robots = []
  for robot_id in range(robots_count):
    robots.append(Robot(robot_id))

  common_urdf = rospy.get_param('robot_description', None)
  if not common_urdf:
    rospy.logerr('No URDF description for common robot')
    return

  for robot in robots:
    robot.get_target_joint_names(common_urdf)
  

  rospy.loginfo('Spinning')
  rospy.spin()


if __name__ == '__main__':
  main(sys.argv)
