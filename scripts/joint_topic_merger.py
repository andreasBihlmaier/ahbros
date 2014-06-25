#!/usr/bin/env python

import roslib
import rospy
import sys
import argparse

#TODO
# take any number of sensor_msgs/JointState topics
# merge them on single topic
# rename single joints according to one urdf model (i.e. ur5, lwr2 -> or_table_...ur5...lwr2)


#def main(args):
#  parser = argparse.ArgumentParser()
#  parser.add_argument('-f', '--freq', type=float, default=10, help='Frequency TFs are republished (default: 10 Hz)')
#  parser.add_argument('target_from_tf', help='Published Base Frame')
#  parser.add_argument('target_to_tf', help='Published Target Frame')
#  parser.add_argument('source_from_tf', help='Read Base Frame')
#  parser.add_argument('source_to_tf', help='Read Target Frame')
#  args = parser.parse_args(rospy.myargv()[1:])
#  target_from_tf, target_to_tf, source_from_tf, source_to_tf = args.target_from_tf, args.target_to_tf, args.source_from_tf, args.source_to_tf
#
#  rospy.init_node('tf_alias_node', anonymous=True)
#  tf_listener = tf.TransformListener()
#  tf_broadcaster = tf.TransformBroadcaster()
#  tf_listener.waitForTransform(source_from_tf, source_to_tf, rospy.Time(), rospy.Duration(4.0))
#
#  rospy.loginfo('Spinning')
#  r = rospy.Rate(args.freq)
#  while not rospy.is_shutdown():
#    position, quaternion = tf_listener.lookupTransform(source_from_tf, source_to_tf, rospy.Time())
#    #print('Publishing %s -> %s (= %s -> %s): position=%s quaternion=%s' % (target_from_tf, target_to_tf, source_from_tf, source_to_tf, position, quaternion))
#    tf_broadcaster.sendTransform(position, quaternion, rospy.get_rostime(), target_to_tf, target_from_tf)
#    r.sleep()
#
#
#if __name__ == '__main__':
#  main(sys.argv)
