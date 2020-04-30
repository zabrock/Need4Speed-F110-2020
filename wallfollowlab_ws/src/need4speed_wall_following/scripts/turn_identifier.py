#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
from need4speed_gap_finding.msg import gaps
from need4speed_wall_following.msg import TurnsPossible
import pdb
pub = rospy.Publisher('turns_possible', TurnsPossible, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = rospy.get_param("/turn_identifier_node/min_distance")
MAX_DISTANCE = rospy.get_param("/turn_identifier_node/max_distance")
MIN_ANGLE = rospy.get_param("/turn_identifier_node/min_angle")
MAX_ANGLE = rospy.get_param("/turn_identifier_node/max_angle")
ANGLE_INCREMENT = np.degrees(rospy.get_param("/turn_identifier_node/angle_increment"))
scan_msg = LaserScan()

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):

  # Make sure the input angle is within acceptable range
  if angle > MAX_ANGLE:
    angle = MAX_ANGLE
  elif angle < MIN_ANGLE:
    angle = MIN_ANGLE

  # Find the index where we should be looking in the LIDAR data for the
  # given angle
  offset_angle = angle - MIN_ANGLE
  range_idx = int(round(offset_angle/ANGLE_INCREMENT))
  # Make sure the index is actually within the length of the LIDAR ranges
  if range_idx < 0:
    range_idx = 0
  elif range_idx >= len(data.ranges):
    range_idx = len(data.ranges) - 1

  # Return the value at the given angle
  value = data.ranges[range_idx]
  if value > MAX_DISTANCE:
    value = MAX_DISTANCE
  elif value < MIN_DISTANCE:
    value = MIN_DISTANCE

  return value

# a: distance from LIDAR to wall at angle theta
# b: distance from LIDAR to wall at "zero" angle (depending on side)
# theta: angle between distances a and b, in degrees
# Outputs the angle alpha in degrees between the car's x-axis and the wall
def getAlpha(a,b,theta):
  # Calculate numerator and denominator in inverse tangent quantity
  num = a*np.cos(np.radians(theta)) - b
  den = a*np.sin(np.radians(theta))
  # Return alpha as defined by Equation (1) in the lab writeup
  return np.degrees(np.arctan(num/den))

# a: distance from LIDAR to wall at angle theta
# b: distance form LIDAR to wall at "zero" angle (depending on side)
# theta: angle between distances a and b, in degrees
# lookahead_distance: projected lookahead distance of car
# Output: D_t+1, or the future-projected distance from the wall
def getFutureDistance(a,b,theta,lookahead_dist):
  # Get the angle from the car's x-axis to the wall
  alpha = getAlpha(a,b,theta)
  # Return the estimated future distance
  return b*np.cos(np.radians(alpha)) + lookahead_dist*np.sin(np.radians(alpha))

def filter_lidar(data):
  # Filter the ranges in data with a simple moving average filter
  filtered_ranges = np.zeros(len(data.ranges))
  filter_width = rospy.get_param("/turn_identifier_node/filter_width")
  # Check that filter_width is valid value; must be odd so that
  # the width on either side of the value to be replaced is equal
  if not filter_width % 2 or not filter_width > 0:
    raise ValueError("ROS parameter filter_width must be positive and odd")
  half_width = (filter_width-1)/2

  for i in range(0,len(data.ranges)):
    min_idx = max(0,i-half_width)
    max_idx = min(i+half_width,len(data.ranges)-1)
    # Add one to max_idx since Python runs only to 1:n for [1:n+1]
    # indexing and add one to difference in indices for correct averaging
    filtered_ranges[i] = sum(data.ranges[min_idx:max_idx+1])/(max_idx-min_idx+1)

  data.ranges = filtered_ranges
  return data

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def laserCallback(msg):
  global scan_msg
  scan_msg = msg

def gapCallback(gap_msg):
  global scan_msg
  MAX_GAP_DISTANCE = rospy.get_param("/turn_identifier_node/max_gap_distance")
  MIN_GAP_WIDTH = rospy.get_param("/turn_identifier_node/min_gap_width")
  THETA = rospy.get_param("/turn_identifier_node/theta")
  MIN_CROSS_THRESH = rospy.get_param("/turn_identifier_node/cross_product_threshold")
  CENTER_DIST_THRESH = rospy.get_param("/turn_identifier_node/center_distance_threshold")

  if rospy.get_param("/turn_identifier_node/filter_lidar"):
    scan_msg = filter_lidar(scan_msg)

  # Get unit vector and point representing left and right walls
  b = getRange(scan_msg,180)
  a = getRange(scan_msg,180-THETA)
  alpha = getAlpha(a,b,THETA)
  leftWallVec = (np.cos(np.radians(alpha)), np.sin(np.radians(alpha)))
  leftWallPt = (0, b)
  b = getRange(scan_msg,0)
  a = getRange(scan_msg,THETA)
  alpha = getAlpha(a,b,THETA)
  rightWallVec = (np.cos(np.radians(alpha)),np.sin(np.radians(alpha)))
  rightWallPt = (0, -b)
  left_possible = False
  right_possible = False
  center_possible = False
  for gap_width, gap_center in zip(gap_msg.gap_widths, gap_msg.gap_centers):
    if gap_width < MIN_GAP_WIDTH:
      continue

    left_gap_vector = getGapVector(leftWallPt,gap_center)
    left_cross_prod = getCrossProduct(leftWallVec,left_gap_vector)
    right_gap_vector = getGapVector(rightWallPt, gap_center)
    right_cross_prod = getCrossProduct(rightWallVec,right_gap_vector)
    # Calculate distance to gap; if greater than max_distance,
    # check if between two walls to determine that center following is possible 
    if gap_center.x**2 + gap_center.y**2 > CENTER_DIST_THRESH**2:
      #if left_cross_prod < MIN_CROSS_THRESH and right_cross_prod > -MIN_CROSS_THRESH:
      #  center_possible = True
      center_possible = True
    # Check gap against left wall vector first
    elif gap_center.x**2 + gap_center.y**2 < MAX_GAP_DISTANCE**2:
      if left_cross_prod > MIN_CROSS_THRESH:
        rospy.loginfo("Left turn detected")
        left_possible = True
      elif right_cross_prod < -MIN_CROSS_THRESH:
        rospy.loginfo("Right turn detected")
        right_possible = True
  

  msg = TurnsPossible()
  if left_possible and center_possible and right_possible:
    msg.turns_possible = TurnsPossible.LEFT_AND_CENTER_AND_RIGHT
  elif left_possible and center_possible:
    msg.turns_possible = TurnsPossible.LEFT_AND_CENTER
  elif center_possible and right_possible:
    msg.turns_possible = TurnsPossible.CENTER_AND_RIGHT
  elif left_possible and right_possible:
    msg.turns_possible = TurnsPossible.LEFT_AND_RIGHT
  elif left_possible:
    msg.turns_possible = TurnsPossible.LEFT
  elif right_possible:
    msg.turns_possible = TurnsPossible.RIGHT
  elif center_possible:
    msg.turns_possible = TurnsPossible.CENTER
  else:
    msg.turns_possible = TurnsPossible.NONE
  pub.publish(msg)

def getLeftWallVector(scan_msg):
  b = getRange(scan_msg,180)
  a = getRange(scan_msg,180-THETA)
  alpha = getAlpha(a,b,THETA)
  return ( np.cos(np.radians(alpha)), np.sin(np.radians(alpha)) )

def getGapVector(wall_pt, gap_loc):
  return (gap_loc.x - wall_pt[0], gap_loc.y - wall_pt[1])

def getCrossProduct(vec1, vec2):
  return vec1[0]*vec2[1] - vec1[1]*vec2[0]

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
	rospy.init_node('turn_identifier_node', anonymous = True)
	scan_sub = rospy.Subscriber("scan", LaserScan, laserCallback)
        gap_sub = rospy.Subscriber("lidar_gaps", gaps, gapCallback)
	rospy.spin()
