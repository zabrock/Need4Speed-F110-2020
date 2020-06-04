#!/usr/bin/env python

import rospy
from race.msg import drive_param
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 
import copy

# Publishers for tracking error
max_error_pub = rospy.Publisher('max_tracking_error', Float64, queue_size=1)
sqd_avg_error_pub = rospy.Publisher('squared_average_tracking_error', Float64, queue_size=1)

max_error = 0
sum_sqd_error = 0
n = 0

# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def callback(msg):
    global max_error
    global sum_sqd_error
    global n

    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    # First find the point closest to the vehicle
    distances = []
    for point in path_points:
        distances.append(dist((x,y),(point[0],point[1])))
    min_idx = distances.index(min(distances))
    closest_pt = path_points[min_idx]
    a = distances[min_idx]
    # Next, get the second closest point
    if min_idx == 0:
        if distances[len(distances)-1] < distances[1]:
            second_closest_pt = path_points[len(distances)-1]
            c = distances[len(distances)-1]
        else:
            second_closest_pt = path_points[1]
            c = distances[1]
    elif min_idx == len(distances)-1:
        if distances[0] < distances[len(distances)-2]:
            second_closest_pt = path_points[0]
            c = distances[0]
        else:
            second_closest_pt = path_points[len(distances)-2]
            c = distances[len(distances)-2]
    else:
        if distances[min_idx-1] < distances[min_idx+1]:
            second_closest_pt = path_points[min_idx-1]
            c = distances[min_idx-1]
        else:
            second_closest_pt = path_points[min_idx+1]
            c = distances[min_idx+1]
            
    # Now use Heron's formula to get the error with respect to the path
    b = dist(closest_pt,second_closest_pt)
    s = (a + b + c)/2.0
    A = np.sqrt(s*(s-a)*(s-b)*(s-c))
    error = A/b
    # Update running error values
    if error > max_error:
        max_error = error
    sum_sqd_error = sum_sqd_error + error**2
    n = n + 1
    
    # Publish error values
    max_error_pub.publish(Float64(max_error))
    sqd_avg_error_pub.publish(Float64(sum_sqd_error/n))

if __name__ == '__main__':
    global path_points
    rospy.init_node('tracking_error_node')
    rospy.Subscriber('/odom', Odometry, callback, queue_size=1)

    # Import waypoints.csv into a list (path_points)
    dirname = os.path.dirname(__file__)
    waypoint_file = rospy.get_param('/pure_pursuit_node/waypoint_filename','levine-waypoints.csv')
    filename = os.path.join(dirname, '../waypoints/' + waypoint_file)
    with open(filename) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    
    # Turn path_points into a list of floats to eliminate the need for casts in the code below.
    path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]

    rospy.spin()
