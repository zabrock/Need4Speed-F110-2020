#!/usr/bin/env python

import rospy
import math
import numpy as np
from need4speed_wall_following.msg import TurnState
from race.msg import drive_param

# Publisher for turning state
pub = rospy.Publisher("car_turn_state", TurnState, queue_size=10)

# Variable to track previously published turning state 
turn_state_msg_old = TurnState()


def callback(msg):
  global turn_state_msg_old
  # Get turning threshold from parameter server
  TURN_INIT_THRESH = rospy.get_param("/turn_tracker_node/turn_init_threshold")
  TURN_COMPLETE_THRESH = rospy.get_param("/turn_tracker_node/turn_complete_threshold")
  new_msg = TurnState()
  state_change = False
  if not turn_state_msg_old.turn_initiated and abs(msg.angle) > TURN_INIT_THRESH:
    new_msg.turn_initiated = True
    new_msg.turn_completed = False
    state_change = True
  elif turn_state_msg_old.turn_initiated and abs(msg.angle) < TURN_COMPLETE_THRESH:
    new_msg.turn_completed = True
    new_msg.turn_initiated = False
    state_change = True

  if state_change:
    turn_state_msg_old = new_msg
    pub.publish(new_msg)
  

if __name__ == '__main__':
	rospy.init_node('turn_tracker_node', anonymous = True)
	rospy.Subscriber("/drive_parameters", drive_param, callback)
	rospy.spin()
