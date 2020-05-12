#!/usr/bin/env python

import rospy
import math
import numpy as np
from need4speed_wall_following.msg import DriveCommand, TurnsPossible, TurnState, instructions
from std_msgs.msg import Bool

if __name__ == "__main__":
	rospy.init_node('dummy_instruction_node', anonymous = True)
	pub = rospy.Publisher("/next_instruction", DriveCommand, queue_size=10)
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		msg = DriveCommand()
		msg.follow_method = DriveCommand.FOLLOW_CENTER
		msg.velocity = 1.0
		pub.publish(msg)
		r.sleep()
