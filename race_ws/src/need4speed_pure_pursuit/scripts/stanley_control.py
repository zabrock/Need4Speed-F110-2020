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

###########
# GLOBALS #
###########
path_points = []
CT_GAIN = 5
VELOCITY = 6
MAX_VELOCITY = 5
MIN_VELOCITY = 2
DELTA_THRESHOLD = 0.05
VEL_GAIN = -10.0
LOOKAHEAD_DISTANCE = 0.5
VEL_LOOKAHEAD = 1.5
KAPPA_THRESHOLD = 0.05
THETA_ERROR_PREV = 0.0
K_D_THETA = 4
        
# Publisher for 'drive_parameters' (speed and steering angle)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Publisher for lookahead distance
lookahead_pub = rospy.Publisher('lookahead_distance', Float64, queue_size=1)


#############
# FUNCTIONS #
#############
    
# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

# Input data is PoseStamped message from topic /odom.
# Runs pure pursuit and publishes velocity and steering angle.
def callback(msg):
    global LOOKAHEAD_DISTANCE
    global VELOCITY
    global THETA_ERROR_PREV

    # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

    # 1. Determine the current location of the vehicle (we are subscribed to /odom)
    # Hint: Read up on Odometry message type in ROS to determine how to extract x, y, and yaw. Make sure to convert quaternion to euler angle.
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    quat = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    (roll, pitch, yaw) = euler_from_quaternion(quat)

    # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
    # First find the point closest to the vehicle
    distances = []
    for point in path_points:
        distances.append(dist((x,y),(point[0],point[1])))
    min_idx = distances.index(min(distances))
    closest_pt = path_points[min_idx]
    # Then find the next point in the path that is >= the lookahead distance
    goal_idx = []
    for i in range(min_idx,len(distances)):
        if dist((closest_pt[0], closest_pt[1]),(path_points[i][0], path_points[i][1])) >= LOOKAHEAD_DISTANCE:
            goal_idx = copy.deepcopy(i)
            break
    if goal_idx != 0 and not goal_idx:
        for i in range(0, min_idx):
            if dist((closest_pt[0], closest_pt[1]),(path_points[i][0], path_points[i][1])) >= LOOKAHEAD_DISTANCE:
                goal_idx = copy.deepcopy(i)
                break
    if goal_idx != 0 and not goal_idx:
        print('Unexpected goal index finding')

    closest_pt = path_points[goal_idx]
    if goal_idx == len(distances)-1:
        second_closest_pt = path_points[0]
        c = distances[0]
    else:
        second_closest_pt = path_points[goal_idx+1]
        c = distances[goal_idx+1]
    a = distances[goal_idx]

    vel_idx = []
    for i in range(min_idx,len(distances)):
        if dist((closest_pt[0], closest_pt[1]),(path_points[i][0], path_points[i][1])) >= VEL_LOOKAHEAD:
            vel_idx = copy.deepcopy(i)
            break
    if vel_idx != 0 and not vel_idx:
        for i in range(0, min_idx):
            if dist((closest_pt[0], closest_pt[1]),(path_points[i][0], path_points[i][1])) >= VEL_LOOKAHEAD:
                vel_idx = copy.deepcopy(i)
                break
    if vel_idx != 0 and not vel_idx:
        print('Unexpected velocity index finding')

    # Transform the closest point to vehicle coordinates. 

    # The following calculations come from the knowledge that p0 = T01p1, where p0
    # is a point in frame {0}, p1 is the same point in frame {1}, and T01 is the
    # transformation matrix that describes frame {1} in frame {0}. If T01 is written as:
    #     T01 = [R01 p01
    #             0   1]
    # then the inverse of the transformation matrix is as follows:
    #     inv(T01) = [R01' -R01'p01
    #                  0       1]
    # where ' denotes the matrix transpose. If frame {1} is taken as the vehicle's
    # coordinate frame and {0} is the map frame in which the path is defined, we
    # can find the goal point in vehicle coordinates by p1 = inv(T01)p0.

    # Point in world coordinates
    x_g_w = closest_pt[0]
    y_g_w = closest_pt[1]

    # Point in car coordinates
    x_g = x_g_w*np.cos(yaw) + y_g_w*np.sin(yaw) - (x*np.cos(yaw) + y*np.sin(yaw))
    y_g = -x_g_w*np.sin(yaw) + y_g_w*np.cos(yaw) + (x*np.sin(yaw) - y*np.cos(yaw))

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle and published to the 'drive_param' topic.

    # Our car's coordinate frame is rotated 90 degrees from that in the paper,
    # so curvature is actually 2y/l^2.
    # "l" is given by the distance to the goal point
    kappa = 2*abs(y_g)/(distances[vel_idx]**2)

    # Trim the velocity based on curvature
    if kappa > KAPPA_THRESHOLD:
        VELOCITY = MAX_VELOCITY + (kappa-KAPPA_THRESHOLD)*VEL_GAIN
    else:
        VELOCITY = MAX_VELOCITY
    if VELOCITY < MIN_VELOCITY:
        VELOCITY = MIN_VELOCITY

    # Now use Heron's formula to get the error with respect to the path
    b = dist(closest_pt,second_closest_pt)
    s = (a + b + c)/2.0
    A = np.sqrt(s*(s-a)*(s-b)*(s-c))
    ct_error = A/b

    if y_g < 0:
        ct_error = -ct_error


    # Calculate heading error
    theta_p = np.arctan2(second_closest_pt[1]-closest_pt[1],second_closest_pt[0]-closest_pt[0])
    theta_error = theta_p - yaw
    theta_error = (theta_error + np.pi) % (2 * np.pi) - np.pi # Wrap to (-pi, pi)

    
    # Calculate control steering angle
    delta = theta_error + K_D_THETA*(theta_error - THETA_ERROR_PREV) + np.arctan(CT_GAIN*ct_error/VELOCITY)
    print('error vals: theta ct')
    print(theta_error)
    print(ct_error)
    THETA_ERROR_PREV = theta_error
    
    # Trim the velocity based on steering angle
    #if delta > DELTA_THRESHOLD:
    #    VELOCITY = MAX_VELOCITY + (delta-DELTA_THRESHOLD)*VEL_GAIN
    #else:
    #    VELOCITY = MAX_VELOCITY
    #
    #if VELOCITY < MIN_VELOCITY:
    #    VELOCITY = MIN_VELOCITY
    

    delta = np.clip(delta, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
    msg = drive_param()
    msg.velocity = VELOCITY
    msg.angle = delta
    pub.publish(msg)

if __name__ == '__main__':
    global path_points

    # Import waypoints.csv into a list (path_points)
    dirname = os.path.dirname(__file__)
    waypoint_file = rospy.get_param('/stanley_node/waypoint_filename','levine-waypoints.csv')
    filename = os.path.join(dirname, '../waypoints/' + waypoint_file)
    print(filename)
    with open(filename) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    
    # Turn path_points into a list of floats to eliminate the need for casts in the code below.
    path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]

    rospy.init_node('stanley_node')
    rospy.Subscriber('/pf/pose/odom', Odometry, callback, queue_size=1)

    rospy.spin()
