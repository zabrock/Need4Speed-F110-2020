#!/usr/bin/env python

import rospy
from race.msg import drive_param
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 
import copy

#############
# CONSTANTS #
#############

LOOKAHEAD_DISTANCE = 2 # meters
VELOCITY = 3 # m/s
MIN_VELOCITY = 3 # m/s
MAX_VELOCITY = 5 # m/s
MIN_LOOKAHEAD = 1 # meters
MAX_LOOKAHEAD = 3 # meters
VEL_GAIN = -15.0 # m/s
LOOKAHEAD_GAIN = 20.0
KAPPA_THRESHOLD = 0.1


###########
# GLOBALS #
###########
path_points = []
        
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

    # Wait until we have a path to follow
    if not path_points:
      return

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
    closest_point = path_points[min_idx]
    
    # Then find the next point in the path that is >= the lookahead distance
    goal_idx = []
    for i in range(min_idx,len(distances)):
        if distances[i] >= LOOKAHEAD_DISTANCE:
            goal_idx = copy.deepcopy(i)
            break
    if goal_idx != 0 and not goal_idx:
        for i in range(0, min_idx):
            if distances[i] >= LOOKAHEAD_DISTANCE:
                goal_idx = copy.deepcopy(i)
                break
    if goal_idx != 0 and not goal_idx:
        # Default to the end of the path
        goal_idx = len(distances)-1

    if goal_idx < 0:
        # Default to driving straight in hopes that things will work out later
        msg = drive_param()
        msg.velocity = MIN_VELOCITY
        msg.angle = 0.0
        pub.publish(msg)
        return
    goal_point = path_points[goal_idx]


    # 3. Transform the goal point to vehicle coordinates. 

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

    # Goal in world coordinates
    x_g_w = goal_point[0]
    y_g_w = goal_point[1]

    # Goal in car coordinates
    x_g = x_g_w*np.cos(yaw) + y_g_w*np.sin(yaw) - (x*np.cos(yaw) + y*np.sin(yaw))
    y_g = -x_g_w*np.sin(yaw) + y_g_w*np.cos(yaw) + (x*np.sin(yaw) - y*np.cos(yaw))

    # 4. Calculate the curvature = 1/r = 2x/l^2
    # The curvature is transformed into steering wheel angle and published to the 'drive_param' topic.

    # Our car's coordinate frame is rotated 90 degrees from that in the paper,
    # so curvature is actually 2y/l^2.
    # "l" is given by the distance to the goal point
    kappa = 2*abs(y_g)/(distances[goal_idx]**2)

    # Trim the velocity based on curvature
    if kappa > KAPPA_THRESHOLD:
        velocity = MAX_VELOCITY + (kappa-KAPPA_THRESHOLD)*VEL_GAIN
    else:
        velocity = MAX_VELOCITY
    if velocity < MIN_VELOCITY:
        velocity = MIN_VELOCITY

    # Trim the lookahead distance based on velocity
    LOOKAHEAD_DISTANCE = MAX_LOOKAHEAD - kappa*LOOKAHEAD_GAIN
    if LOOKAHEAD_DISTANCE < MIN_LOOKAHEAD:
        LOOKAHEAD_DISTANCE = MIN_LOOKAHEAD
    if LOOKAHEAD_DISTANCE > MAX_LOOKAHEAD:
        LOOKAHEAD_DISTANCE = MAX_LOOKAHEAD
    
    wheelbase = rospy.get_param('/racecar_simulator/wheelbase',0.3302)
    # Following comes from geometry
    angle = np.arctan(kappa*wheelbase)
    # Change angle's sign depending on whether y_g is to the left or right of the vehicle
    if y_g < 0:
        angle = -angle

    angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    msg = drive_param()
    msg.velocity = velocity
    msg.angle = angle
    pub.publish(msg)

    # Also publish the lookahead distance
    lookahead_pub.publish(Float64(LOOKAHEAD_DISTANCE))

def path_callback(msg):
    global path_points
    if (not path_points) or rospy.get_param('/pure_pursuit_node/dynamic_path',False):
        path_points = [(float(pose.pose.position.x), float(pose.pose.position.y)) for pose in msg.poses]
        print("updated path_points")
    
if __name__ == '__main__':
    global path_points

    rospy.init_node('pure_pursuit')
    rospy.Subscriber('/pf/pose/odom', Odometry, callback, queue_size=1)
    rospy.Subscriber('/desired_path', Path, path_callback, queue_size=1)

    rospy.spin()

