#!/usr/bin/python3

import numpy as np 
import turtle
import argparse
import time
import rospy
import math
import cv2
import sensor_msgs.point_cloud2 as pc2
import struct
import ctypes

from copy import deepcopy

from maze import Maze, Particle, WeightedDistribution, weight_gaussian_kernel
from nav_msgs.msg import Odometry, OccupancyGrid
from detection_clustering import DetectionClustering
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from sensor_msgs.msg import PointCloud2


class Particle_Filter(object):
    def __init__(self, window_width, window_height, num_particles,\
         sensor_limit_ratio, grid_height, grid_width, lane,\
              num_rows, num_cols, wall_prob, random_seed,\
                   robot_speed, kernel_sigma, particle_show_frequency):
        self.sensor_limit = sensor_limit_ratio * max(grid_height * num_rows, grid_width * num_cols)
        self.window = turtle.Screen()
        self.window.setup(width = window_width, height = window_height)
        self.world = Maze(grid_height = grid_height, grid_width = grid_width,\
             lane = lane, num_rows = num_rows, num_cols = num_cols,\
                  wall_prob = wall_prob, random_seed = random_seed)
        self.particle_show_frequency = particle_show_frequency
        self.num_particles = num_particles
        self.prev_pose = [0, 0, 0]
        self.cur_pose = [0, 0, 0]
        self.counter = 0
        self.tracker = 0
        self.cloud_data = list()
        self.particles = list()
        self.map = None
        self.lane = lane
        self.permissible_space = [[lane, lane, num_rows - lane, num_cols - lane]]
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.doors = [[60, 0], [70, 0], [90, 0], [150, 0], [220, 0], [90, 40], [120, 40], [150, 40], \
                        [238, 48], [238, 170], [238, 310], [200, 170], [200, 155], \
                        [60, 328], [50, 328], [150, 290], [100, 290], [60, 290], [170, 328], \
                        [40, 150], [40, 140]]


        # Initilizing particles randomly 
        self.initialize_particles()

        self.distribution = WeightedDistribution(particles = self.particles)
        # Display the map with particles
        time.sleep(1)
        self.world.show_maze()
        
        # Initializing pretrained objects
        self.detected = {}        
        rospy.init_node('update_particles', anonymous=True)
        self.rate = rospy.Rate(10)  
        self.localize()
    
    def localize(self):
        counter = 1
        #self.filtering()
        while(1):
            # Particle filtering
            rospy.Subscriber('/rtabmap/odom', Odometry, self.callback)
            rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, self.collect_cloud)
            if(self.eucDist() > 1):
                total_weight = 0
                total_lt = 0
                for particle in self.particles:
                    permissible = particle.try_move(maze=self.world, cur_pose=self.cur_pose,\
                        prev_pose=self.prev_pose)
                    if(permissible == 0):
                        particle.weight = 0
                        distribution = WeightedDistribution(particles = self.particles)
                        self.selectParticle(distribution, particle = particle)
                        while(not self.check_permissible_space(x=particle.x, y=particle.y)):
                            self.selectParticle(distribution, particle = particle)      
                    else:
                        particle.lifetime += 1
                        particle.weight *= 2
                    total_weight += particle.weight
                print("particle moved: Now filtering------------------", total_weight)
                if(total_weight == 0):
                    total_weight = 1e-8              
                total_lt = 0
                for particle in self.particles:
                    particle.weight /= total_weight
                    total_lt += particle.lifetime
                
                print('Filtering started')
                self.filtering()
                counter = 0
                print("filtering ended")
                print('Num particles left: ', len(self.particles))   
                self.prev_pose = list(self.cur_pose)
                print('----------------------------------------------------------------------')
            self.world.show_particles(particles = self.particles, show_frequency = self.particle_show_frequency)
            self.world.show_doors(doors=self.doors, show_frequency=self.particle_show_frequency)
            self.world.clear_objects()
            counter += 1
            

    def eucDist(self):
         deltaTrans = math.sqrt((self.cur_pose[0] - self.prev_pose[0]) ** 2 + (self.cur_pose[1] - self.prev_pose[1]) ** 2)
         return deltaTrans
    
    def selectParticle(self, distribution, particle):
        particle_new = distribution.random_select()
        while(particle_new == None):
            particle_new = distribution.random_select()
        particle_new.add_noise()
        particle.x = particle_new.x
        particle.y = particle_new.y 
        particle.heading = particle_new.heading
        particle.init_x = particle_new.init_x
        particle.init_y = particle_new.init_y 
        particle.init_heading = particle_new.init_heading
        particle.weight = particle_new.weight
        particle.lifetime = particle_new.lifetime
        return particle_new

    def filtering(self):
        #return 0
        if(len(self.cloud_data) == 0):
            return 0        
        print("Particle weights measuring start-------")
        particle_weight_total = 0
        min_wt = 0
        min_lt = np.inf
        counter = 0
        for particle in self.particles:
            particle.weight = particle.read_cloud_weight(cloud_data=self.cloud_data)
            min_wt = min(min_wt, particle.weight)
            min_lt = min(min_lt, particle.lifetime)
            particle_weight_total += particle.weight
            counter += 1

        print('Particle weights measured-----------------', particle_weight_total, min_wt)
        min_lt = max(0, min_lt-1)
        print("Min lifetime: ", min_lt)
        particle_weight_total = 0
        for particle in self.particles:
            if(particle.active):
                particle.weight -= min_wt
                #particle.lifetime -= min_lt
                particle.weight *= particle.lifetime
            else:
                particle.lifetime = 0
            particle_weight_total +=  particle.weight
            
        print('Particle weights calculated------------------', particle_weight_total)  
        
        if particle_weight_total == 0:
            particle_weight_total = 1e-8

        particle_new_total = 0
        for particle in self.particles:
            particle.weight /= particle_weight_total
            #particle.weight *= particle.lifetime
            particle_new_total += particle.weight
            #print('particle weight: ', particle.weight)

        print('Weights normalized', particle_new_total)

        self.distribution = WeightedDistribution(particles = self.particles)
        particle_new_list = list()
        countNone = 0
        particle_new_total = 0
        for i in range(self.num_particles):
            self.sample_particle(particle_new_list = particle_new_list)
            particle_new_total += particle_new_list[i].weight  
        self.particles = particle_new_list
        print("New Particles ready: ", particle_new_total)

        if(particle_new_total < 1):
            return 0
        for particle in self.particles:
            particle.weight /= particle_new_total

    def sample_particle(self, particle_new_list):
        particle = Particle(x = 0, y = 0, maze = self.world, sensor_limit = self.sensor_limit)
        self.selectParticle(self.distribution, particle)
        while(not self.check_permissible_space(x=particle.x, y=particle.y)):
            self.selectParticle(self.distribution, particle)
        particle.active = True
        particle.lifetime = 1
        particle_new_list.append(particle)



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
        self.cur_pose[0] = data.pose.pose.position.x
        self.cur_pose[1] = data.pose.pose.position.y
        self.cur_pose[2] = self.quaternion_to_euler(data.pose.pose.orientation.x, \
            data.pose.pose.orientation.y, data.pose.pose.orientation.z, \
            data.pose.pose.orientation.w)
    
    def collect_cloud(self, data):
        self.cloud_data = list()
        for p in pc2.read_points(data, skip_nans=True):
            rgb = p[3]
            s = struct.pack('>f' ,rgb)
            i = struct.unpack('>l',s)[0]
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            #print (r,g,b)
            if(r <= 20 and g <= 20 and b <= 20):
                self.cloud_data.append([p[1], p[0]])

    def quaternion_to_euler(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)
        return Z    

    # ------------------------------------------------------------------------

if __name__ == '__main__':

    window_width = 500
    window_height = 500
    num_particles = 2000
    sensor_limit_ratio = 0.3
    grid_height = 10
    grid_width = 10
    num_rows = 33
    num_cols = 24
    wall_prob = 0.25
    random_seed = 100
    robot_speed = 10
    kernel_sigma = 500
    particle_show_frequency = 1
    lane = 4
    Particle_Filter(window_width = window_width, window_height = window_height,\
         num_particles = num_particles, sensor_limit_ratio = sensor_limit_ratio,\
              grid_height = grid_height, grid_width = grid_width,\
                   lane = lane, num_rows = num_rows, num_cols = num_cols,\
                        wall_prob = wall_prob, random_seed = random_seed,\
                             robot_speed = robot_speed, kernel_sigma = kernel_sigma,\
                                  particle_show_frequency = particle_show_frequency)