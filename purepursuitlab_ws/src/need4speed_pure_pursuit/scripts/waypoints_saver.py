#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

# Open a file to write waypoints to
home = expanduser('~')
file = open(strftime(home+'/waypoints-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

# Publisher for path messages
path_pub = rospy.Publisher('/saved_path', Path, queue_size=1)

# Initialize path message
saved_path = Path()

def pfCallback(msg):
    global saved_path

    # Get heading from quaternion
    quat = np.array([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    rpy = tf.transformations.euler_from_quaternion(quat)

    # Yaw angle is third in euler
    file.write('%f, %f, %f\n' % (msg.pose.pose.position.x,msg.pose.pose.position.y,rpy[2]) )

    # Copy pose into saved_path
    saved_path.header = msg.header
    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = msg.pose.pose
    saved_path.poses.append(pose_stamped)

    # Publish the saved path
    path_pub.publish(saved_path)


def shutdown():
    file.close()
 
def listener():
    rospy.init_node('waypoints_saver', anonymous=True)
    rospy.Subscriber('pf/pose/odom', Odometry, pfCallback)
    rospy.spin()

if __name__ == '__main__':
    # Handler for shutdowns
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
