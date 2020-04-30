#!/usr/bin/env python

import rospy
import math
import numpy as np
from need4speed_wall_following.msg import DriveCommand, TurnsPossible, TurnState, instructions

class instructionFollower:
  def __init__(self):
    self.instruction_sub = rospy.Subscriber("/explicit_instructions", instructions, self.instructionCallback)
    self.drive_cmd_sub = rospy.Subscriber("/next_instruction", DriveCommand, self.driveCommandCallback)
    self.turn_state_sub = rospy.Subscriber("/car_turn_state", TurnState, self.turnStateCallback)
    self.turns_possible_sub = rospy.Subscriber("/turns_possible", TurnsPossible, self.turnsPossibleCallback)
    self.pub = rospy.Publisher("drive_command", DriveCommand, queue_size=10)
    self.instruction_info_pub = rospy.Publisher("executing_instruction", DriveCommand, queue_size=10)

    self.saved_instructions = instructions()
    self.saved_turn_state = TurnState()
    self.saved_turns_possible = TurnsPossible()
    self.saved_next_instruction = DriveCommand()

  def instructionCallback(self, msg):
    self.saved_instructions = msg

  def driveCommandCallback(self, msg):
    self.saved_next_instruction = msg

  def turnStateCallback(self, msg):
    self.saved_turn_state = msg

  def turnsPossibleCallback(self, msg):
    self.saved_turns_possible = msg

  def publishCommand(self):
    msg = DriveCommand()
    msg.velocity = DriveCommand.EMPTY_VELOCITY
    msg.follow_method = DriveCommand.EMPTY_FOLLOW_METHOD

    if rospy.get_param("/instruction_follower_node/use_explicit_instructions"):
      msg.velocity = 2.0
      msg.follow_method = DriveCommand.FOLLOW_LEFT

    print("publishing drive command")
    self.pub.publish(msg)
    
    self.instruction_info_pub.publish(msg)

  def loop(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.publishCommand()
      rate.sleep()
  
    
if __name__ == '__main__':
	rospy.init_node('instruction_follower_node', anonymous = True)
	follower = instructionFollower()
	follower.loop()
