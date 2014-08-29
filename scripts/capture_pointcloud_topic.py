#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import argparse
import subprocess
import os
import shutil
from sensor_msgs.msg import PointCloud2

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-t', '--topic', default='points', help='Pointcloud topic')
  parser.add_argument('-o', '--output_file', default='/tmp/points.pcd', help='Pointcloud destination file')
  parser.add_argument('-p', '--pcd_prefix', default='/tmp', help='Prefix to temporarily store pcd files')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('capture_pointcloud_topic', anonymous=True)

  rospy.loginfo('Waiting for pointcloud msg on %s' % args.topic)
  # Set to line buffering mode
  process_args = ['stdbuf', '--output=L', '--error=L', 'rosrun', 'pcl_ros', 'pointcloud_to_pcd', 'input:=%s' % args.topic, '_prefix:=%s/' % args.pcd_prefix]
  pointcloud_to_pcb_process = subprocess.Popen(process_args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  pcd_written = False
  while True:
    output = pointcloud_to_pcb_process.stdout.readline()
    if 'Data saved to' in output:
      if not pcd_written:
        pcd_written = True
        pcd_file = output.split()[-1].split('.')[0] + '.pcd'
      else:
        pointcloud_to_pcb_process.terminate()
        rm_pcd_file = output.split()[-1].split('.')[0] + '.pcd'
        pointcloud_to_pcb_process.wait()
        os.remove(rm_pcd_file)
        break
  shutil.move(pcd_file, args.output_file)



if __name__ == '__main__':
  main()
