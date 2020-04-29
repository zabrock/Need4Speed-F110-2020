#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np
import math

# Persistent variables to hold previous error and time values for
# derivative calculation
previous_error = 0.0
previous_time = 0.0
previous_de_dt = 0.0

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Speed control algorithm based on steering angle.
# steer_angle: steer tire angle computed by PID controller, radians
# Outputs velocity in meters per second
def calculate_velocity(steer_angle):
  # Set speed thresholds and speed values
  MAX_SPEED = rospy.get_param("/control_node/max_speed")
  MED_SPEED = rospy.get_param("/control_node/med_speed")
  MIN_SPEED = rospy.get_param("/control_node/min_speed")
  MAX_SPEED_THRESH = rospy.get_param("/control_node/max_speed_threshold")
  MED_SPEED_THRESH = rospy.get_param("/control_node/med_speed_threshold")

  # Follow the step-like velocity thresholds as defined in the lab writeup
  if abs(steer_angle) < MAX_SPEED_THRESH:
    return MAX_SPEED
  elif abs(steer_angle) < MED_SPEED_THRESH:
    return MED_SPEED
  else:
    return MIN_SPEED

def filter_error(error):
  # Apply a first-order low-pass filter to the error value
  # to smooth out the noise
  alpha = rospy.get_param("/control_node/lp_filter_coefficient")
  return alpha*previous_error + (1-alpha)*error
  
# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(msg):
  global previous_error
  global previous_de_dt
  global previous_time
  # Extract error and get current time for derivative calculation
  error = filter_error(msg.data)
  time = rospy.Time.now().to_sec()
  # Don't calculate derror_dt if no previous value was saved
  if previous_time == 0.0:
    derror_dt = 0
  else:
    derror_dt = (error - previous_error)/(time - previous_time)

  KP = rospy.get_param("/control_node/kp")
  KD = rospy.get_param("/control_node/kd")
  alpha_d = rospy.get_param("/control_node/deriv_filter_coefficient")

  # Use PD control to determine steer tire angle
  # Apply low-pass filter to derivative to reduce high-frequency gain
  pid_output = KP*error + KD*(alpha_d*previous_de_dt + (1-alpha_d)*derror_dt)
  angle = math.radians(pid_output)    #convert the angle to radians if not already in radians
  angle = np.clip(angle, -0.4189, 0.4189)  # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

  # Calculate velocity based on steer tire angle
  vel = calculate_velocity(angle)

  # Save error and time for next iteration
  previous_error = error
  previous_de_dt = derror_dt
  previous_time = time

  msg = drive_param()
  msg.velocity = vel
  msg.angle = angle
  pub.publish(msg)

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
  rospy.init_node('pid_controller_node', anonymous=True)
  rospy.Subscriber("pid_error", Float64, control_callback)
  rospy.spin()

