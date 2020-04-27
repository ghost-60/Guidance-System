#!/usr/bin/python3

import numpy as np 
import rospy
#import ros_numpy
import sensor_msgs.point_cloud2 as pc2
import cv2
import struct
import ctypes

from sensor_msgs.msg import PointCloud2

def callback(data):
    print('checkpoint')
    t = pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)
    x = 0
    y = 0
    z = 0
    count = 0
    gridmap = np.zeros((600, 600))
    max_x = 0
    min_x = 0
    max_y = 0
    min_y = 0
    door_count = 0
    for p in pc2.read_points(data, skip_nans=True):
        x += p[0]
        y += p[1]
        # max_x = max(max_x, p[0])
        # max_y = max(max_y, p[1])
        # min_x = min(min_x, p[0])
        # min_y = min(min_y, p[1])
        count += 1
        rgb = p[3]
        s = struct.pack('>f' ,rgb)
        i = struct.unpack('>l',s)[0]
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        #print (r,g,b)
        if(r <= 20 and g <= 20 and b <= 20):
            gridmap[(int(p[0])+10)*5, (int(p[1])+10)*5] = 200
            door_count += 1
    #print(count, x/count, y/count, z/count)
    #print(min_x, max_x, min_y, max_y)
    print(door_count)
    cv2.imshow('im', gridmap)
    cv2.waitKey(1)
    

rospy.init_node('pointcloud_particles', anonymous=True)
rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, callback)
rospy.spin()