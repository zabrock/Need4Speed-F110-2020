#!/usr/bin/env python

import rospy
from need4speed_wall_following.msg import instructions
import csv


pub = rospy.Publisher('explicit_instructions', instructions, queue_size=10)

instructions = instructions()
def explicit():
	#change path if needed
	with open('/home/stark/f110_ws/src/xu_wall_following/explicit_instructions/instructions.csv') as file:
		reader = csv.reader(file, delimiter=',')
		for ins in reader:
			instructions.turns.append(ins[0])
			instructions.speeds.append(float(ins[1]))

	rate = rospy.Rate(5)
	while not rospy.is_shutdown():
		rospy.loginfo(instructions)
		pub.publish(instructions)
		rate.sleep()

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
        rospy.init_node('read_instruction_node', anonymous=True)
	explicit()
	rospy.spin()
