#!/usr/bin/python

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

class Particle_Filter(object):
    def __init__(self, window_width, window_height, num_particles, sensor_limit_ratio, grid_height, grid_width, lane, num_rows, num_cols, wall_prob, random_seed, robot_speed, kernel_sigma, particle_show_frequency):
        self.sensor_limit = sensor_limit_ratio * max(grid_height * num_rows, grid_width * num_cols)
        self.window = turtle.Screen()
        self.window.setup(width = window_width, height = window_height)
        self.world = Maze(grid_height = grid_height, grid_width = grid_width, lane = lane, num_rows = num_rows, num_cols = num_cols, wall_prob = wall_prob, random_seed = random_seed)
        self.particle_show_frequency = particle_show_frequency
        self.num_particles = num_particles
        self.prev_pose = [0, 0, 0]
        self.cur_pose = [0, 0, 0]
        self.counter = 0
        self.tracker = 0
        self.dc = None
        self.particles = list()
        self.map = None
        self.lane = lane
        self.permissible_space = [[lane, lane, num_rows - lane, num_cols - lane]]
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.doors = [[60, 0], [70, 0], [150, 0], [220, 0], [90, 40], [120, 40], [150, 40], \
                        [238, 48], [238, 170], [238, 310], [200, 170], [200, 155], \
                        [60, 328], [50, 328], [150, 290], [100, 290], [60, 290], \
                        [40, 150], [40, 140]]


        # Initilizing particles randomly 
        self.initialize_particles()

        self.particles = list()
        # for i in range(10):
        #     self.particles.append(Particle(x = 2, y = 340, maze = self.world, heading = 180, sensor_limit = self.sensor_limit))
        for i in range(5):
            self.particles.append(Particle(x = 220, y = 40, maze = self.world, heading = 0, sensor_limit = self.sensor_limit))
        
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
        counter = 0
        fil_count = 0
        detection_size = 0
        #self.filtering()
        while(1):
            # Particle filtering
           
            if(self.eucDist() > 1):
                #print("First: ", self.eucDist)
                print(self.cur_pose)
                total_weight = 0
                for particle in self.particles:
                    permissible = particle.try_move(maze=self.world, cur_pose=self.cur_pose,\
                        prev_pose=self.prev_pose)
                    if(permissible == 0):
                        particle.weight = 0
                        #print("P1: ", particle.x, particle.y)
                        particle_new = self.selectParticle()
                        particle.x = particle_new.x
                        particle.y = particle_new.y
                        particle.heading = particle_new.heading
                        particle.weight = particle_new.weight
                        particle.lifetime = particle_new.lifetime
                        while(not self.check_permissible_space(x=particle.x, y=particle.y)):
                            particle_new = self.selectParticle()
                            particle.x = particle_new.x
                            particle.y = particle_new.y
                            particle.heading = particle_new.heading
                            particle.weight = particle_new.weight
                            particle.lifetime = particle_new.lifetime
                        #print("P2: ", particle.x, particle.y)
                    else:
                        particle.lifetime += 1
                        particle.weight *= 2
                    total_weight += particle.weight
                print("particle moved: Now filtering------------------")
                if(total_weight == 0):
                    total_weight = 1e-8    
                for particle in self.particles:
                    particle.weight /= total_weight
                print('Filtering started')
                self.filtering()
                counter = 0
                print("filtering ended")
                print('Num particles left: ', len(self.particles))
                for particle in self.particles:
                    print('Pos: ', particle.x, particle.y)    
                self.prev_pose = list(self.cur_pose)
                print('----------------------------------------------------------------------')
                if(fil_count % 15 == 0):
                    fil_count = 0
                fil_count += 1
            self.world.show_particles(particles = self.particles, show_frequency = self.particle_show_frequency)
            self.world.show_doors(doors=self.doors, show_frequency=self.particle_show_frequency)
            rospy.Subscriber('/rtabmap/odom', Odometry, self.callback)
            rospy.Subscriber('/cluster_decomposer/centroid_pose_array', PoseArray, self.collect)
            self.world.clear_objects()
            counter += 1
            

    def eucDist(self):
         deltaTrans = math.sqrt((self.cur_pose[0] - self.prev_pose[0]) ** 2 + (self.cur_pose[1] - self.prev_pose[1]) ** 2)
         return deltaTrans
    
    def selectParticle(self):
        distribution = WeightedDistribution(particles = self.particles)
        particle_new = distribution.random_select()
        while(particle_new == None):
            particle_new = distribution.random_select()
        particle_new.add_noise()
        return particle_new

    def filtering(self):
        print('Start filtering doors')
        self.dc = DetectionClustering(self.detected.copy(), min_samples=10)
        print('Doors filtered')
        return 0
        if('door' in self.dc.clusters):
            # self.pub_msg.data = list(np.ndarray.flatten(np.asarray(self.dc.clusters['door'])))
            # self.pub.publish(self.pub_msg)
            # self.rate.sleep()
            print('Doors: ', len(self.dc.clusters['door']))
            readings_robot = self.zed_sensor_reading()
            #print("WTF AAAAAAA....I am sane, yes, totally")
            if(len(readings_robot) == 0):
                return 0

            print("Particle weights measuring start-------")
            particle_weight_total = 0
            min_wt = 0
            min_lt = np.inf
            for particle in self.particles:
                readings_particle = particle.read_sensor(maze=self.world)
                particle.weight = weight_gaussian_kernel(x1 = readings_robot, x2 = readings_particle, particle = particle)
                min_wt = min(min_wt, particle.weight)
                min_lt = min(min_lt, particle.lifetime)
                #particle_weight_total += particle.weight

            print('Particle weights measured-----------------')
            for particle in self.particles:
                if(particle.active):
                    particle.weight += min_wt
                    particle.lifetime -= (min_lt - 1)
                    particle.weight *= particle.lifetime
                particle_weight_total +=  particle.weight
                #print('particle weight: ', particle.weight)
            print('Particle weights calculated------------------')  
            
            if particle_weight_total == 0:
                particle_weight_total = 1e-8

            for particle in self.particles:
                particle.weight /= particle_weight_total
            
            print('Weights normalized')

            self.distribution = WeightedDistribution(particles = self.particles)
            print("Distribution ready")
            particles_new_list = list()
            for i in range(self.num_particles):
                particle_new = self.distribution.random_select()
                while(particle_new == None):
                    particle_new = self.distribution.random_select()
                particle_new.add_filter_noise()
                while(not self.check_permissible_space(x=particle_new.x, y=particle_new.y)):
                    particle_new = self.distribution.random_select()
                    while(particle_new == None):
                        particle_new = self.distribution.random_select()
                    particle_new.add_filter_noise()
                particle_new.active = True
                particles_new_list.append(particle_new)
            self.particles = particles_new_list
            print("New Particles ready")

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
        
    def quaternion_to_euler(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)
        return Z

    # Collecting objects --------------------------------------------------
    def update_key(self, key, val):
        if(key in self.detected):
            self.detected[key].append(val)
        else:
            self.detected[key] = [val]

    def collect(self, msg):
        for i, pose in enumerate(msg.poses):
            if(pose != Pose() and i == 0):
                pos = pose.position
                val = [pos.x, pos.y, pos.z]
                key = 'door'
                self.update_key(key, val)
    
    def zed_sensor_reading(self):
        readings_robot = []
        for p in self.dc.clusters['door']:
            dist = math.sqrt((p[0] - self.cur_pose[0])**2 + (p[1] - self.cur_pose[1])**2)
            delRot = math.atan2(p[1] - self.cur_pose[1], p[0] - self.cur_pose[0]) - self.cur_pose[2]
            #delRot = (delRot + (int(delRot / (2*np.pi)) + 1) * 2*np.pi) % 360
            if(dist < 10):
                readings_robot.append([dist, delRot])
        return readings_robot
    # ------------------------------------------------------------------------

if __name__ == '__main__':

    window_width = 500
    window_height = 500
    num_particles = 5
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