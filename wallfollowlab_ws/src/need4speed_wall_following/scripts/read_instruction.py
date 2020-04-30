#!/usr/bin/env python

import rospy
from need4speed_wall_following.msg import instructions
import csv


pub = rospy.Publisher('explicit_instructions', instructions, queue_size=10)


def explicit():
	#change path if needed
	instruction_path = rospy.get_param('/read_instruction_node/instruction_path')
	instruction_set = instructions()
	with open(instruction_path) as file:
		reader = csv.reader(file, delimiter=',')
		for ins in reader:
			instruction_set.turns.append(ins[0])
			instruction_set.speeds.append(float(ins[1]))

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rospy.loginfo(instruction_set)
		pub.publish(instruction_set)
		rate.sleep()

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
        rospy.init_node('read_instruction_node', anonymous=True)
	explicit()
	rospy.spin()
