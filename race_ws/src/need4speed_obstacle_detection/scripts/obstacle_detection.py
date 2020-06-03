#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import rospy
import numpy as np
import math
from scipy import ndimage

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


# gloval variable
scanData = None
odomData = None
countPrint = 0

def scan_callback(data):
    global scanData 
    scanData = data
    
def odom_callback(data):
    global odomData 
    odomData = data



def computeObstacle():
    rate = rospy.Rate(10)             # speed of this func (in Hz)
    countProgramStarting = 0
    countProgramStarting_Ready = 4    # to prevent errors at start
    while not rospy.is_shutdown():
        if countProgramStarting < countProgramStarting_Ready:
            countProgramStarting += 1
        else:
            # --- main code starts here ---
            # compute laserScanData length and angle array
            lengthScanData = len(scanData.ranges)
            scanDataAngle = np.zeros(lengthScanData)
            for n in range(lengthScanData):
                if n < lengthScanData / 2:
                    scanDataAngle[n] = scanData.angle_min + scanData.angle_increment * n
                else:
                    nFromEnd = lengthScanData - 1 - n
                    scanDataAngle[n] = scanData.angle_max - scanData.angle_increment * nFromEnd

            # distinguish obstacles by making array of ones and zeros
            # : 0 means path, 1 means obstacle
            thresholdMaxDist = 4.5    # adjust for detection performance (roadWidth = approx 3.0)
            thresholdDiff    = 0.8    # adjust for detection performance
            scanDataRangesClip = np.clip(scanData.ranges, 0.0, thresholdMaxDist)  # limit the amount of error
            scanDataDiff1 = np.ediff1d(scanDataRangesClip, to_end  =0.0)          # differentiate the array
            scanDataDiff2 = np.ediff1d(scanDataDiff1     , to_begin=0.0)
            boolArray = np.zeros(lengthScanData, dtype=int)
            for n in range(lengthScanData):
                if scanDataRangesClip[n] < thresholdMaxDist:
                    boolArray[n] = 1
                if abs(scanDataDiff2[n]) > thresholdDiff:
                    boolArray[n] = 0            # makes a notch (to separate a cluster later)

            # cluster the laserScanData    
            clusterData,numCluster = ndimage.label(boolArray)

            # print message if it is near obstacle/curve
            global countPrint 
            if numCluster > 3:
                print('obstacle/curve detected!   (msgNumber:',countPrint)
                countPrint += 1

            # get obstale data from laserScan
            obstacleDistance = []
            obstacleAngle = []
            for n in range(lengthScanData):
                if clusterData[n] != 0:
                    obstacleDistance.append(scanData.ranges[n])
                    obstacleAngle.append(scanDataAngle[n])
            lengthObstacle = len(obstacleDistance) 

            # convert obstacle coord from poloar to rectangular
            xObstacle_ScanCoord = np.zeros(lengthObstacle)
            yObstacle_ScanCoord = np.zeros(lengthObstacle)
            for n in range(lengthObstacle):
                xObstacle_ScanCoord[n] = obstacleDistance[n] * math.cos(obstacleAngle[n])
                yObstacle_ScanCoord[n] = obstacleDistance[n] * math.sin(obstacleAngle[n])

            # get car position in Odometry Coord
            xCar_OdomCoord = odomData.pose.pose.position.x
            yCar_OdomCoord = odomData.pose.pose.position.y
            [roll,pitch,yaw] = euler_from_quaternion([ \
                odomData.pose.pose.orientation.x, odomData.pose.pose.orientation.y, \
                odomData.pose.pose.orientation.z, odomData.pose.pose.orientation.w  \
                ])
            thetaCar_OdomCoord = yaw 

            # get obstacle position in Odometry Coord
            xObstacle_OdomCoord = np.zeros(lengthObstacle)
            yObstacle_OdomCoord = np.zeros(lengthObstacle)
            for n in range(lengthObstacle):
                xObstacle_OdomCoord[n] \
                    = xObstacle_ScanCoord[n] * math.cos(thetaCar_OdomCoord) \
                    - yObstacle_ScanCoord[n] * math.sin(thetaCar_OdomCoord) \
                    + xCar_OdomCoord
                yObstacle_OdomCoord[n] \
                    = xObstacle_ScanCoord[n] * math.sin(thetaCar_OdomCoord) \
                    + yObstacle_ScanCoord[n] * math.cos(thetaCar_OdomCoord) \
                    + yCar_OdomCoord
   
            # get marker
            topic = 'visualization_marker'
            markerPub = rospy.Publisher(topic, Marker, queue_size=10)
            marker = Marker()
            marker.header.frame_id = "Obstacle"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "my_namespace"
            marker.id = 10
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = 4.0
            marker.pose.position.y = 2.0 
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0    
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            markerPub.publish(marker)
        rate.sleep()    # last line of else statement




# start this ROS node.
if __name__ == '__main__':
    rospy.init_node('obstacle_detection_node', anonymous = True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.Subscriber("odom", Odometry,  odom_callback)
    try: 
        computeObstacle()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
