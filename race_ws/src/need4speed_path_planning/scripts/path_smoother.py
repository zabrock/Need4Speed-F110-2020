#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import csv
import os 
import copy

N_INTERP = 100
POLY_ORDER = 4

pub = rospy.Publisher('/desired_path', Path, queue_size=1)

def callback(msg):
  # Extract the x,y coordinates from the path
  x = [pose.pose.position.x for pose in msg.poses]
  y = [pose.pose.position.y for pose in msg.poses]
  
  # Fit a polynomial to the data
  t = np.linspace(0,1,len(x))
  zx = np.polyfit(t,x,POLY_ORDER)
  fx = np.poly1d(zx)
  zy = np.polyfit(t,y,POLY_ORDER)
  fy = np.poly1d(zy)
  
  # Calculate the fit points to the data
  t_new = np.linspace(0,1,N_INTERP)
  x_new = fx(t_new)
  y_new = fy(t_new)
  
  # Now put the newly fit path into a message and publish
  new_msg = Path()
  new_msg.header.frame_id = "/map"
  for (x,y) in zip(x_new, y_new):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    new_msg.poses.append(pose)
    
  pub.publish(new_msg)

if __name__ == "__main__":
  rospy.init_node('path_smoother')
  rospy.Subscriber('/rrt_path', Path, callback, queue_size=1)

  rospy.spin()
