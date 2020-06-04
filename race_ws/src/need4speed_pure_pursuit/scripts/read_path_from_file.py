#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 
import copy
        
# Publisher for path
pub = rospy.Publisher('desired_path', Path, queue_size=1)
    
if __name__ == '__main__':
    rospy.init_node('path_reader')
    # Import waypoints.csv into a list (path_points)
    dirname = os.path.dirname(__file__)
    waypoint_file = rospy.get_param('/pure_pursuit_node/waypoint_filename','levine-waypoints.csv')
    filename = os.path.join(dirname, '../waypoints/' + waypoint_file)
    with open(filename) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    
    # Turn path_points into a list of floats to eliminate the need for casts in the code below.
    path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]

    # Now set up a path to publish out
    msg = Path()
    msg.header.frame_id = "/map"
    for point in path_points:
      pose = PoseStamped()
      pose.pose.position.x = point[0]
      pose.pose.position.y = point[1]
      msg.poses.append(pose)
    
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
      pub.publish(msg)
      rate.sleep()

