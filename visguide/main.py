#!/usr/bin/python3

import numpy as np 
import turtle
import argparse
import time
import rospy
import math
import cv2

from maze import Maze, Particle, WeightedDistribution, weight_gaussian_kernel
from nav_msgs.msg import Odometry, OccupancyGrid
from detection_clustering import DetectionClustering
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from turtlebot3_slam_3d.msg import floatArray, float2dArray

class Particle_Filter(object):
    def __init__(self, window_width, window_height, num_particles, sensor_limit_ratio, grid_height, grid_width, num_rows, num_cols, wall_prob, random_seed, robot_speed, kernel_sigma, particle_show_frequency):
        self.sensor_limit = sensor_limit_ratio * max(grid_height * num_rows, grid_width * num_cols)
        self.window = turtle.Screen()
        self.window.setup(width = window_width, height = window_height)
        self.world = Maze(grid_height = grid_height, grid_width = grid_width, num_rows = num_rows, num_cols = num_cols, wall_prob = wall_prob, random_seed = random_seed)
        self.particle_show_frequency = particle_show_frequency
        self.num_particles = num_particles
        self.prev_x = 0
        self.prev_y = 0
        self.live_x = 0
        self.live_y = 0
        self.counter = 0
        self.tracker = 0
        self.dc = None
        self.particles = list()
        self.map = None
        self.permissible_space = [[3, 3, 30, 24]]
        self.num_rows = num_rows
        self.num_cols = num_cols

        # Initilizing particles randomly 
        self.initialize_particles()
        self.distribution = WeightedDistribution(particles = self.particles)
        # Display the map with particles
        time.sleep(1)
        self.world.show_maze()
        
        # Initializing pretrained objects
        self.detected = {}        
        rospy.init_node('update_particles', anonymous=True)
        self.pub = rospy.Publisher('detected_objects', Float32MultiArray, queue_size=10)
        self.pub_msg = Float32MultiArray()
        self.rate = rospy.Rate(10)
        
        self.localize()
    
    def localize(self):
        
        
        readings_robot = []

        while(1):
            
            dx = self.live_x
            dy = self.live_y
            
            # Particle filtering
            self.filtering()
            
            for particle in self.particles:
                permissible = particle.try_move(maze=self.world, dx=dx*10, dy=dy*10)
                if(permissible == 0):
                    particle_new = self.distribution.random_select()
                    particle.x = particle_new.x
                    particle.y = particle_new.y
                    particle.heading = particle_new.heading
                    particle.weight /= 2
                    
            self.prev_x = self.live_x
            self.prev_y = self.live_y
            self.world.show_particles(particles = self.particles, show_frequency = self.particle_show_frequency)
            rospy.Subscriber('/visguide/zed_node/odom', Odometry, self.callback)
            rospy.Subscriber('/cluster_decomposer/centroid_pose_array', PoseArray, self.collect)
            self.world.clear_objects()

    def filtering(self):
        self.dc = DetectionClustering(self.detected.copy(), min_samples=10)
        if('door' in self.dc.clusters):

            self.pub_msg.data = list(np.ndarray.flatten(np.asarray(self.dc.clusters['door'])))
            self.pub.publish(self.pub_msg)
            self.rate.sleep()
            
            readings_robot = self.zed_sensor_reading()
            
            if(len(readings_robot) == 0):
                return 0
            
            particle_weight_total = 0
            for particle in self.particles:
                readings_particle = particle.read_sensor(maze=self.world)
                particle.weight = weight_gaussian_kernel(x1 = readings_robot, x2 = readings_particle)
                particle_weight_total += particle.weight
                
            if particle_weight_total == 0:
                particle_weight_total = 1e-8

            for particle in self.particles:
                particle.weight /= particle_weight_total
            
            self.distribution = WeightedDistribution(particles = self.particles)
            particles_new = list()
            for i in range(num_particles):
                particle = self.distribution.random_select()
                particles_new.append(Particle(x = particle.x, y = particle.y, maze = self.world, heading = particle.heading, sensor_limit = self.sensor_limit, noisy = True))
            
            self.particles = particles_new

    def initialize_particles(self):
        for i in range(self.num_particles):
            x = np.random.uniform(0, self.world.width)
            y = np.random.uniform(0, self.world.height)
            while(not self.check_permissible_space(x=x, y=y)):
                x = np.random.uniform(0, self.world.width)
                y = np.random.uniform(0, self.world.height)
            self.particles.append(Particle(x = x, y = y, maze = self.world, sensor_limit = self.sensor_limit))
    
    def check_permissible_space(self, x, y):
        if(x <= 1 or y <= 1 or x >= (self.num_cols * 10 - 2) or y >= (self.num_rows * 10 - 2)):
            return False

        for p in self.permissible_space:
            x1 = p[0] * 10
            y1 = p[1] * 10
            x2 = p[2] * 10
            y2 = p[3] * 10
            if(x >= x1 - 1 and y >= y1 - 1 and x <= y2 + 1 and y <= x2 + 1):
                return False
        return True
        
    def callback(self, data):
        #self.tracker += 1
        self.live_x = data.pose.pose.position.x
        self.live_y = data.pose.pose.position.y

    # Collecting objects --------------------------------------------------
    def update_key(self, key, val):
        if(key in self.detected):
            self.detected[key].append(val)
        else:
            self.detected[key] = [val]

    def collect(self, msg):
        for i, pose in enumerate(msg.poses):
            if(pose != Pose()):
                pos = pose.position
                val = [pos.x, pos.y, pos.z]
                key = 'door'
                self.update_key(key, val)
    
    def zed_sensor_reading(self):
        readings_robot = []
        for p in self.dc.clusters['door']:
            dist = math.sqrt(math.pow((p[0] - self.live_x), 2) + math.pow((p[1] - self.live_y), 2))
            if(dist < 10):
                readings_robot.append(dist)
        return readings_robot
    # ------------------------------------------------------------------------

if __name__ == '__main__':

    window_width = 500
    window_height = 500
    num_particles = 2000
    sensor_limit_ratio = 0.3
    grid_height = 10
    grid_width = 10
    num_rows = 33
    num_cols = 27
    wall_prob = 0.25
    random_seed = 100
    robot_speed = 10
    kernel_sigma = 500
    particle_show_frequency = 1

    Particle_Filter(window_width = window_width, window_height = window_height, num_particles = num_particles, sensor_limit_ratio = sensor_limit_ratio, grid_height = grid_height, grid_width = grid_width, num_rows = num_rows, num_cols = num_cols, wall_prob = wall_prob, random_seed = random_seed, robot_speed = robot_speed, kernel_sigma = kernel_sigma, particle_show_frequency = particle_show_frequency)