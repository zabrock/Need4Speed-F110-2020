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
    self.request_instruction_pub = rospy.Publisher("request_instruction", Bool, queue_size=10)

    self.saved_instructions = instructions()
    self.saved_turn_state = TurnState()
    self.saved_turns_possible = TurnsPossible()
    self.saved_next_instruction = DriveCommand()

    self.center_start_time = 0.0
    self.old_instruction = DriveCommand()

    self.executing_instruction = False
    self.executing_turn = False
    self.following_center = False
    self.received_first_instruction = False

  def instructionCallback(self, msg):
    self.saved_instructions = msg

  def driveCommandCallback(self, msg):
    self.saved_next_instruction = msg
    if msg.follow_method == DriveCommand.EMPTY_FOLLOW_METHOD and msg.velocity == DriveCommand.EMPTY_VELOCITY:
      print("finished instructions, restoring default wall following behavior")
    else:
      print("received new instruction: ")
      print(self.saved_next_instruction)
    self.received_first_instruction = True

  def turnStateCallback(self, msg):
    self.saved_turn_state = msg
    if self.executing_turn and self.saved_turn_state.turn_completed:
      print("finished turn")
      self.executing_turn = False
      if self.executing_instruction:
        print("finished instruction")
        self.instruction_complete_pub.publish(Bool(True))
        self.executing_instruction = False

  def turnsPossibleCallback(self, msg):
    self.saved_turns_possible = msg

  def publishCommand(self):
    # Resort to original wall following behavior if no instructions received
    msg = DriveCommand()
    msg.velocity = DriveCommand.EMPTY_VELOCITY
    msg.follow_method = DriveCommand.EMPTY_FOLLOW_METHOD

    
    if rospy.get_param("/instruction_follower_node/use_explicit_instructions"):
      if not self.received_first_instruction:
        # Request first instruction
        print("Requesting first instruction")
        self.request_instruction_pub.publish(Bool(True))
      else:
        # Get next instruction
        msg = self.explicitInstructionAlgorithm()

    self.pub.publish(msg)
    self.old_instruction = msg
    
    if self.executing_instruction or self.following_center:
      self.instruction_info_pub.publish(self.saved_next_instruction)
    else:
      msg = DriveCommand()
      msg.velocity = DriveCommand.EMPTY_VELOCITY
      msg.follow_method = DriveCommand.EMPTY_FOLLOW_METHOD
      self.instruction_info_pub.publish(msg)

  def explicitInstructionAlgorithm(self):
    follow_method = self.saved_next_instruction.follow_method
    velocity = self.saved_next_instruction.velocity
    turns_possible = self.saved_turns_possible.turns_possible
    
    if follow_method == DriveCommand.EMPTY_FOLLOW_METHOD and velocity == DriveCommand.EMPTY_VELOCITY:
      # Perform original wall following behavior
      return self.saved_next_instruction

    # If car was previously executing a turn, continue to publish it
    if self.executing_turn:
      # It's possible the car missed one of the turns in leadup to corridor;
      # if so and we see it, modify to follow the instruction
      if not self.executing_instruction:
        if turns_possible == TurnsPossible.LEFT_AND_RIGHT or turns_possible == TurnsPossible.LEFT_AND_CENTER_AND_RIGHT:
          print("instruction now possible, changing behavior")
          cmd = self.old_instruction
          cmd.follow_method = follow_method
          cmd.velocity = velocity
          self.executing_instruction = True
          return cmd
        elif (turns_possible == TurnsPossible.LEFT and follow_method == DriveCommand.FOLLOW_LEFT) or (turns_possible == TurnsPossible.RIGHT and follow_method == DriveCommand.FOLLOW_RIGHT):
          print("instruction now possible, changing behavior")
          cmd = self.old_instruction
          cmd.follow_method = follow_method
          cmd.velocity = velocity
          self.executing_instruction = True
          return cmd
        else:
          return self.old_instruction
      else:
        return self.old_instruction

    # If car was previously following center by instruction, check if we can assume center following is complete
    if self.following_center == True:
      center_follow_duration = rospy.Time.now() - self.center_start_time
      CENTER_FOLLOW_MIN_TIME = rospy.get_param("/instruction_follower_node/center_follow_min_time")
      if self.saved_turns_possible.turns_possible == TurnsPossible.CENTER and center_follow_duration.to_sec() > CENTER_FOLLOW_MIN_TIME:
        print("finished center instruction")
        self.following_center = False
        self.instruction_complete_pub.publish(Bool(True))
      else:
        cmd = self.old_instruction
        # Make sure we're on the right heading for what the car sees.
        # It's possible that the car missed either a left or right gap during leadup to the corridor; 
        # if so, check here if we see it and modify to center following if that's the case
        turns_possible = self.saved_turns_possible.turns_possible
        if turns_possible == TurnsPossible.LEFT_AND_CENTER_AND_RIGHT:
          cmd.follow_method = DriveCommand.FOLLOW_CENTER
        return cmd
    
    # Set up corridor following behavior
    cmd = DriveCommand()
    cmd.follow_method = rospy.get_param("/instruction_follower_node/corridor_follow_method")
    cmd.velocity = rospy.get_param("/instruction_follower_node/corridor_velocity")
    
    # If only center gap is detected, keep following it with corridor behavior
    if self.saved_turns_possible.turns_possible == TurnsPossible.CENTER or self.saved_turns_possible.turns_possible == TurnsPossible.NONE:
      return cmd

    # Otherwise, try to determine which way to go
    if follow_method == DriveCommand.FOLLOW_LEFT:
      # Check if a left turn appears possible
      valid = (turns_possible == TurnsPossible.LEFT_AND_CENTER_AND_RIGHT) or (turns_possible == TurnsPossible.LEFT_AND_CENTER) or (turns_possible == TurnsPossible.LEFT_AND_RIGHT) or (turns_possible == TurnsPossible.LEFT)
      if valid:
        print("following left turn instruction")
        cmd.follow_method = follow_method
        cmd.velocity = velocity
        self.executing_turn = True
        self.executing_instruction = True
        self.instruction_start_time = rospy.Time.now()
#      elif turns_possible == TurnsPossible.CENTER_AND_RIGHT:
#        # Stay left to keep center
#        cmd.follow_method = DriveCommand.FOLLOW_LEFT
#        print("left not possible, defaulting to drive straight")
      else:
        # Default to a right turn
        cmd.follow_method = DriveCommand.FOLLOW_RIGHT
        self.executing_turn = True
        print("left not possible, performing right turn")

    elif follow_method == DriveCommand.FOLLOW_RIGHT:
      # Check if a right turn appears possible
      valid = (turns_possible == TurnsPossible.LEFT_AND_CENTER_AND_RIGHT) or (turns_possible == TurnsPossible.CENTER_AND_RIGHT) or (turns_possible == TurnsPossible.LEFT_AND_RIGHT) or (turns_possible == TurnsPossible.RIGHT)
      if valid:
        print("following right turn instruction")
        cmd.follow_method = follow_method
        cmd.velocity = velocity
        self.executing_instruction = True
        self.executing_turn = True
#      elif turns_possible == TurnsPossible.LEFT_AND_CENTER:
#        # Stay right to keep center
#        cmd.follow_method = DriveCommand.FOLLOW_RIGHT
#        print("right not possible, defaulting to drive straight")
      else:
        # Default to a left turn
        cmd.follow_method = DriveCommand.FOLLOW_LEFT
        self.executing_turn = True
        print("right not possible, performing left turn")

    elif follow_method == DriveCommand.FOLLOW_CENTER:
      if turns_possible == TurnsPossible.CENTER_AND_RIGHT:
        self.following_center = True
        self.center_start_time = rospy.Time.now()
        # Stay left to keep on center
        print("staying left to follow center instruction")
        cmd.follow_method = DriveCommand.FOLLOW_LEFT
        cmd.velocity = velocity
      elif turns_possible == TurnsPossible.LEFT_AND_CENTER:
        self.following_center = True
        self.center_start_time = rospy.Time.now()
        # Stay right to keep on center
        print("staying right to follow center instruction")
        cmd.follow_method = DriveCommand.FOLLOW_RIGHT
        cmd.velocity = velocity
      elif turns_possible == TurnsPossible.LEFT_AND_CENTER_AND_RIGHT:
        cmd.follow_method = DriveCommand.FOLLOW_CENTER
        cmd.velocity = velocity
        self.following_center = True
        self.center_start_time = rospy.Time.now()
        print("following center instruction")
      elif turns_possible == TurnsPossible.LEFT:
        cmd.follow_method = DriveCommand.FOLLOW_LEFT
        print("straight not possible, turning left")
        self.executing_turn = True
      elif turns_possible == TurnsPossible.RIGHT:
        cmd.follow_method = DriveCommand.FOLLOW_RIGHT
        self.executing_turn = True
        print("straight not possible, turning right")
      elif turns_possible == TurnsPossible.LEFT_AND_RIGHT:
        # Default to left turns
        cmd.follow_method = DriveCommand.FOLLOW_LEFT
        self.executing_turn = True
        print("straight not possible, defaulting left")

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
