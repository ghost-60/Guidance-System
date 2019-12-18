#!/usr/bin/python3

import rospy
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray


class Grid(object):
	def __init__(self):
		self.map = None
		self.pose = [0, 0]
		self.objects = None
		self.have_objects = False
		rospy.init_node('occ_grid_map', anonymous=True)
		rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.callback)
		rospy.Subscriber('/visguide/zed_node/odom', Odometry, self.location)
		rospy.Subscriber('/detected_objects', Float32MultiArray, self.landmarks)
		rospy.spin()

	def landmarks(self, data):
		rows = len(data.data) / 3
		x = np.asarray(data.data)
		self.objects = np.reshape(x, (int(rows), 3))
		self.have_objects = True	

	def callback(self, data):
		self.map = np.reshape(data.data, (data.info.height, data.info.width))
		self.map = np.uint8(self.map)
		x = int(self.pose[0] * 10 - data.info.origin.position.x * 10)
		y = int(self.pose[1] * 10 - data.info.origin.position.y * 10)
		for i in range(x - 3, x + 3):
			for j in range(y - 3, y + 3):
				self.map[j][i] = 150
		
		if(self.have_objects):
			for i in range(self.objects.shape[0]):
				x = int(self.objects[i][0] * 10 - data.info.origin.position.x * 10)
				y = int(self.objects[i][1] * 10 - data.info.origin.position.y * 10)
				for j in range(x-5, x+5):
					for k in range(y-5, y+5):
						if(k >= 0 and k < data.info.height and j >= 0 and j < data.info.width):
							self.map[k][j] = 200

		# print(data.info.origin)
		# print(data.info.height)
		# print(self.pose)
		# print('---------------------')
		cv2.imshow('im', self.map)
		cv2.waitKey(1)
	
	def location(self, data):
		self.pose[0] = data.pose.pose.position.x
		self.pose[1] = data.pose.pose.position.y

if __name__ == '__main__':
	Grid()