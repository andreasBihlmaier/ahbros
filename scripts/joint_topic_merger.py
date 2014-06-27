#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse

from sensor_msgs.msg import JointState

#TODO
# take any number of sensor_msgs/JointState topics
# merge them on single topic
# rename single joints according to one urdf model (i.e. ur5, lwr2 -> or_table_...ur5...lwr2)

class Robot(object):
  def __init__(self, id):
    self.id = id
    self.urdf = rospy.get_param('robot_description%d' % self.id, None)
    if not self.urdf:
      rospy.logerr('No URDF description for robot %d' % self.id)
      return
    self.joint_sub = rospy.Subscriber('joint_states%d' % self.id, JointState, self.on_joint)

  def on_joint(self, msg):
    #print(msg)
    pass


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
    TODO


  rospy.loginfo('Spinning')
  rospy.spin()


if __name__ == '__main__':
  main(sys.argv)
