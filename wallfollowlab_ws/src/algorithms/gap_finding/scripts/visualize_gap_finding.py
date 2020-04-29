#!/usr/bin/env python

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from need4speed_gap_finding.msg import gaps
import rospy

# We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
publisher = rospy.Publisher('/visualization_gap_finding', Marker, queue_size="1")
all_gap_publisher = rospy.Publisher('/visualization_all_gaps', MarkerArray, queue_size="1")

# Input data is Vector3 representing center of largest gap
def callback(data):
    marker = Marker()

# Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
    marker.header.frame_id = "/laser"
    marker.pose.position.x = data.x
    marker.pose.position.y = data.y
    marker.pose.position.z = data.z # or set this to 0

    marker.type = marker.SPHERE

    marker.scale.x = 1.0 # If marker is too small in Rviz can make it bigger here
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # Publish the MarkerArray
    print("Sending marker")
    publisher.publish(marker)

def multi_gap_callback(msg):
    marker_array = MarkerArray()
    for index, gap_center in enumerate(msg.gap_centers):
        marker = Marker()
# Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
        marker.header.frame_id = "/laser"
        marker.pose.position.x = gap_center.x
        marker.pose.position.y = gap_center.y
        marker.pose.position.z = gap_center.z # or set this to 0
    
        marker.type = marker.SPHERE
	marker.id = index
        marker.scale.x = 0.5 # If marker is too small in Rviz can make it bigger here
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        print(marker)
        marker_array.markers.append(marker)

    # Publish the MarkerArray
    print("Sending marker array")
    all_gap_publisher.publish(marker_array)

        

if __name__ == '__main__':
    rospy.init_node('visualize_gap_finding')
    rospy.Subscriber('/gap_center', Vector3, callback)
    rospy.Subscriber('/lidar_gaps', gaps, multi_gap_callback)
    rospy.spin()
