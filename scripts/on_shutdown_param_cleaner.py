#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
import sys
import argparse



def main(args):
  parser = argparse.ArgumentParser()
  parser.add_argument('params', metavar='param', nargs='+', help='Parameters to delete on shutdown')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('on_shutdown_param_cleaner', anonymous=True)

  print('Will delete params %s on shutdown' % args.params)

  try:
    while not rospy.is_shutdown():
      rospy.sleep(1)
  except rospy.exceptions.ROSInterruptException:
    pass

  for param in args.params:
    if rospy.has_param(param):
      rospy.delete_param(param)


if __name__ == '__main__':
  main(sys.argv)
