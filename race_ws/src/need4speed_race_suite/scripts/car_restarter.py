#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, String

pub = rospy.Publisher('/key', String, queue_size = 10)

def callback(msg):
    if msg.data:
        pub.publish(String('n'))
        print('Restarting car')

if __name__ == '__main__':
    rospy.init_node('car_restart_node')
    rospy.Subscriber('/reset_car', Bool, callback, queue_size=1)

    rospy.spin()
