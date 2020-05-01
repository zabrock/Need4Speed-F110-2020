#!/usr/bin/env python
import rospy
from need4speed_wall_following.msg import instructions
from need4speed_wall_following.msg import DriveCommand
from std_msgs.msg import String, Bool
import csv

pub_next_command = rospy.Publisher('next_instruction', DriveCommand, queue_size=10)

global index
global instruction_set
index = 0

#def turnStatus():		# test_1
def instructionCompleteCallback(msg):		
	global index
#	if True:	# test_2	
	if msg.data:
		# Increment index
		index = index + 1
		publishInstruction()
		

def requestInstructionCallback(msg):
	publishInstruction()

def publishInstruction():
	global index
	drive_command = DriveCommand()
	if index < len(instruction_set.turns):	
		drive_command.follow_method = instruction_set.turns[index]
		drive_command.velocity = instruction_set.speeds[index]
		
		rospy.loginfo(drive_command)
	else:
		# Publish out an empty command message since there's no
		# more instructions to follow
		drive_command.follow_method = DriveCommand.EMPTY_FOLLOW_METHOD
		drive_command.velocity = DriveCommand.EMPTY_VELOCITY

	pub_next_command.publish(drive_command)
						 
def readInstructions():
	# Find what path instructions should be read from
	global instruction_set
	instruction_path = rospy.get_param('/read_instruction_node/instruction_path')
	instruction_set = instructions()

	# Read instructions from the file
	with open(instruction_path) as file:
		reader = csv.reader(file, delimiter=',')
		for ins in reader:
			instruction_set.turns.append(ins[0])
			instruction_set.speeds.append(float(ins[1]))
	print(instruction_set)
	# Extract and publish the first command from the list
	drive_command = DriveCommand()
	drive_command.follow_method = instruction_set.turns[0]
	drive_command.velocity = instruction_set.speeds[0]
	pub_next_command.publish(drive_command)
	rospy.loginfo(drive_command)

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
        rospy.init_node('read_instruction_node', anonymous=True)
	readInstructions()
#	turnStatus()	# test_3
	rospy.Subscriber("instruction_complete", Bool, instructionCompleteCallback)
	rospy.Subscriber("request_instruction", Bool, requestInstructionCallback)
	rospy.spin()
