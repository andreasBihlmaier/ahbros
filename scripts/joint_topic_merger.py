#!/usr/bin/env python

from __future__ import print_function

import copy
import roslib
import rospy
import sys
import argparse
import xml.etree.ElementTree as ET
import threading

from sensor_msgs.msg import JointState

class JointTopicAggregator(object):
  def __init__(self):
    self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size = 1)
    self.state_count = 0
    self.current_states = {}
    self.lock = threading.Lock()
    self.sequence = 0


  def publish(self, msg):
    if self.state_count == 0:
      return

    self.lock.acquire()
    for (name_idx, name) in enumerate(msg.name):
      self.current_states[name] = [msg.position[name_idx], msg.velocity[name_idx], msg.effort[name_idx]]

    if len(self.current_states) == self.state_count:
      aggregated_msg = JointState()
      aggregated_msg.header.seq = self.sequence = self.sequence + 1
      aggregated_msg.header.stamp = rospy.Time.now()
      for name in sorted(self.current_states):
        aggregated_msg.name.append(name)
        aggregated_msg.position.append(self.current_states[name][0])
        aggregated_msg.velocity.append(self.current_states[name][1])
        aggregated_msg.effort.append(self.current_states[name][2])
      self.joint_pub.publish(aggregated_msg)
      self.current_states = {}
    self.lock.release()


  def set_states(self, state_count):
    self.state_count = state_count



class Robot(object):
  def __init__(self, id, aggregator = None):
    self.id = id
    self.source_joint_names = []
    self.target_joint_names = []
    self.urdf = rospy.get_param('robot_description%d' % self.id, None)
    self.aggregator = aggregator
    if not self.urdf:
      rospy.logerr('No URDF description for robot %d' % self.id)
      return
    self.name = ET.fromstring(self.urdf).attrib['name']
    rospy.loginfo('id=%d name=%s' % (self.id, self.name))
    self.joint_sub = rospy.Subscriber('joint_states%d' % self.id, JointState, self.on_joint, queue_size = 1)
    if not self.aggregator:
      self.target_joint_pub = rospy.Publisher('joint_states', JointState, queue_size = 1)
    else:
      self.target_joint_pub = self.aggregator


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
      if not target_names:
        target_names = [joint_name for joint_name in all_target_joint_names if joint_name.endswith(src_name)]
      if not target_names and src_name[:-1].endswith('_dof'):
        print('joint with "_dof" suffix, applying special hack for robot_joint_position_controller_gazebo')
        if src_name[-1] == '0':
          target_names = [joint_name for joint_name in all_target_joint_names if joint_name.endswith(src_name[:-5])]
        elif src_name[-1] == '1':
          target_names = [joint_name for joint_name in all_target_joint_names if joint_name.endswith(src_name[:-5] + '__revolute_dummy_joint')]
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
  parser.add_argument('-a', '--aggregate', action='store_true', help='Aggregate all topics into single large JointState message')
  parser.add_argument('robots_count', type=int, help='Number of merged robots')
  args = parser.parse_args(rospy.myargv()[1:])
  robots_count = args.robots_count

  rospy.init_node('joint_topic_merger')

  aggregator = None
  if args.aggregate:
    rospy.loginfo('Aggregation enabled')
    aggregator = JointTopicAggregator()

  robots = []
  for robot_id in range(robots_count):
    robots.append(Robot(robot_id, aggregator))

  common_urdf = rospy.get_param('robot_description', None)
  if not common_urdf:
    rospy.logerr('No URDF description for common robot')
    return

  for robot in robots:
    robot.get_target_joint_names(common_urdf)
  
  if args.aggregate:
    aggregator.set_states(len(rospy.get_param('single_to_composite_joints', {})))

  rospy.loginfo('Spinning')
  rospy.spin()


if __name__ == '__main__':
  main(sys.argv)
