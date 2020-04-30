#!/usr/bin/env python

import rospy
import math
import numpy as np
from need4speed_wall_following.msg import DriveCommand, TurnsPossible, TurnState, instructions
from std_msgs.msg import Bool

class instructionFollower:
  def __init__(self):
    self.instruction_sub = rospy.Subscriber("/explicit_instructions", instructions, self.instructionCallback)
    self.drive_cmd_sub = rospy.Subscriber("/next_instruction", DriveCommand, self.driveCommandCallback)
    self.turn_state_sub = rospy.Subscriber("/car_turn_state", TurnState, self.turnStateCallback)
    self.turns_possible_sub = rospy.Subscriber("/turns_possible", TurnsPossible, self.turnsPossibleCallback)
    self.pub = rospy.Publisher("drive_command", DriveCommand, queue_size=10)
    self.instruction_info_pub = rospy.Publisher("executing_instruction", DriveCommand, queue_size=10)
    self.instruction_complete_pub = rospy.Publisher("instruction_complete", Bool, queue_size=10)

    self.saved_instructions = instructions()
    self.saved_turn_state = TurnState()
    self.saved_turns_possible = TurnsPossible()
    self.saved_next_instruction = DriveCommand()

    self.instruction_start_time = 0.0
    self.old_instruction = DriveCommand()

    self.executing_instruction = False

  def instructionCallback(self, msg):
    self.saved_instructions = msg

  def driveCommandCallback(self, msg):
    self.saved_next_instruction = msg

  def turnStateCallback(self, msg):
    self.saved_turn_state = msg
    if self.executing_instruction and self.saved_turn_state.turn_completed:
      print("finished instruction")
      self.executing_instruction = False
      self.instruction_complete_pub.publish(Bool(True))

  def turnsPossibleCallback(self, msg):
    self.saved_turns_possible = msg

  def publishCommand(self):
    msg = DriveCommand()
    msg.velocity = DriveCommand.EMPTY_VELOCITY
    msg.follow_method = DriveCommand.EMPTY_FOLLOW_METHOD

    if rospy.get_param("/instruction_follower_node/use_explicit_instructions"):
      # Get next instruction
      msg = self.explicitInstructionAlgorithm()

    self.pub.publish(msg)
    self.old_instruction = msg
    
    if msg.velocity == self.saved_next_instruction.velocity and msg.follow_method == self.saved_next_instruction.follow_method:
      self.instruction_info_pub.publish(msg)

  def explicitInstructionAlgorithm(self):
    # If car was previously executing an instruction, continue to publish it
    if self.executing_instruction:
      return self.old_instruction
    
    # Set up default behavior
    cmd = DriveCommand()
    cmd.follow_method = rospy.get_param("/instruction_follower_node/default_follow_method")
    cmd.velocity = rospy.get_param("/instruction_follower_node/default_velocity")
    
    # If only center gap is detected, keep following it with default behavior
    if self.saved_turns_possible.turns_possible == TurnsPossible.CENTER:
      return cmd

    # Otherwise, try to determine which way to go
    follow_method = self.saved_next_instruction.follow_method
    velocity = self.saved_next_instruction.velocity
    turns_possible = self.saved_turns_possible.turns_possible
    
    if follow_method == DriveCommand.FOLLOW_LEFT:
      # Check if a left turn appears possible
      valid = (turns_possible == TurnsPossible.LEFT_AND_CENTER_AND_RIGHT) or (turns_possible == TurnsPossible.LEFT_AND_CENTER) or (turns_possible == TurnsPossible.LEFT_AND_RIGHT) or (turns_possible == TurnsPossible.LEFT)
      print("attempting left turn")
      if valid:
        cmd.follow_method = follow_method
        cmd.velocity = velocity
        self.executing_instruction = True
    elif follow_method == DriveCommand.FOLLOW_RIGHT:
      # Check if a right turn appears possible
      valid = (turns_possible == TurnsPossible.LEFT_AND_CENTER_AND_RIGHT) or (turns_possible == TurnsPossible.CENTER_AND_RIGHT) or (turns_possible == TurnsPossible.LEFT_AND_RIGHT) or (turns_possible == TurnsPossible.RIGHT)
      if valid:
        cmd.follow_method = follow_method
        cmd.velocity = velocity
        self.executing_instruction = True
    elif follow_method == DriveCommand.FOLLOW_CENTER:
      return cmd #TODO: Needs implementation

    return cmd 
      
    
  def loop(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.publishCommand()
      rate.sleep()
  
    
if __name__ == '__main__':
	rospy.init_node('instruction_follower_node', anonymous = True)
	follower = instructionFollower()
	follower.loop()
