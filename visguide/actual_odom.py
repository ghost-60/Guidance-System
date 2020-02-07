#!/usr/bin/python

import rospy
import numpy as np
import cv2

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

class Grid(object):
    def __init__(self):
        self.odomData = None

        self.markerArray = MarkerArray()
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "visguide"
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.id = 0 
        self.marker.text = "testing"
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.scale.z = 1
        self.marker.scale.y = 1
        self.marker.scale.x = 1

        rospy.init_node('actual_odom', anonymous=True)
        self.pub = rospy.Publisher('/actual_odom', Marker, queue_size=30)
        rospy.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped, self.callback)
        rospy.Subscriber('/visguide/zed_node/odom', Odometry, self.location)
        rospy.spin()

    def callback(self, data):      
        self.marker.header.stamp = rospy.Time()
        self.marker.pose = data.pose.pose
        self.marker.pose.orientation.w = 1
        self.pub.publish(self.marker)
    
    def location(self, data):
        self.odomData = data


Grid()