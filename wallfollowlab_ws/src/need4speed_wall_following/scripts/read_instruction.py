#!/usr/bin/env python
import rospy
from need4speed_wall_following.msg import instructions
from need4speed_wall_following.msg import DriveCommand
from std_msgs.msg import String, Bool
import csv

pub_next_command = rospy.Publisher('next_instruction', DriveCommand, queue_size=10)
pub_1st_command = rospy.Publisher('explicit_instructions', instructions, queue_size=10)

global index
index = 0

def turnStatus():		# test_1
#def turnStatus(msg):		
	global index
	drive_command = DriveCommand()
	command = explicitTurns()
	
#	rate = rospy.Rate(1)
#	while not rospy.is_shutdown():
	if index < len(command.turns):
		if True:	# test_2	
#		if msg.data:	
			drive_command.follow_method = command.turns[index]
			drive_command.velocity = command.speeds[index]
				
 			pub_next_command.publish(drive_command)
			rospy.loginfo(drive_command)
			index = index +1
#				rate.sleep()
						 
def explicitTurns():
	#change path if needed
	instruction_path = rospy.get_param('/read_instruction_node/instruction_path')
	instruction_set = instructions()

	with open(instruction_path) as file:
		reader = csv.reader(file, delimiter=',')
		for ins in reader:
			instruction_set.turns.append(ins[0])
			instruction_set.speeds.append(float(ins[1]))

#	rate = rospy.Rate(1)
#	while not rospy.is_shutdown():
#		rospy.loginfo(instruction_set)
		pub_1st_command.publish(instruction_set)
#		rate.sleep()
	
	return instruction_set
# Boilerplate code to start this ROS node.
if __name__ == '__main__':
        rospy.init_node('read_instruction_node', anonymous=True)
	turnStatus()	# test_3
#	rospy.Subscriber("instruction_complete", Bool, turnStatus)
	rospy.spin()
