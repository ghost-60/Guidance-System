#!/usr/bin/python

#WTF IS GOING ON !!!!!!!!!!!!!!!!!!!!!!!!!!

# Oh...I know

import numpy as np 
import turtle
import bisect
import argparse
import random
import math

class Maze(object):

    def __init__(self, grid_height, grid_width, maze = None, num_rows = None, num_cols = None, wall_prob = None, random_seed = None):
        self.grid_height = grid_height
        self.grid_width = grid_width
        self.walls = [[3, 3, 3, 24], [3, 3, 30, 3], [30, 3, 30, 24], [3, 24, 30, 24]] 
        self.walls.append([6, 3, 6, 5])
        self.walls.append([3, 5, 6, 5])
        self.permissible_space = [[3, 3, 30, 24]]
        self.random_maze(num_rows = num_rows, num_cols = num_cols, wall_prob = wall_prob, random_seed = random_seed)

        self.height = self.num_rows * self.grid_height
        self.width = self.num_cols * self.grid_width
        self.landmarks = [[60, 0], [70, 0], [140, 0], [250, 0], [260, 15], \
                        [260, 30], [260, 140], [260, 170], [260, 320], [40, 320], [70, 320], \
                        [70, 30], [120, 30], [140, 30], [230, 160], [230, 170], \
                        [130, 290], [110, 290], [100, 290], [30, 150], [30, 140]]
        
        self.turtle_registration()

    def turtle_registration(self):
        turtle.register_shape('tri', ((-3, -2), (0, 3), (3, -2), (0, 0)))

    def random_maze(self, num_rows, num_cols, wall_prob, random_seed = None):

        self.num_rows = 33
        self.num_cols = 27
        self.maze = np.zeros((self.num_rows, self.num_cols), dtype = np.int8)
        
        # Outer boundary ------------------------------------------

        # left
        for i in range(33):
            self.maze[i, 0] |= 8
        # up
        for i in range(27):
            self.maze[0, i] |= 1
        # right
        for i in range(33):
            self.maze[i, 26] |= 2
        # down
        for i in range(27):
            self.maze[32, i] |= 4

        for x in self.walls:
            if(x[0] == x[2]):
                for i in range(x[1], x[3]):
                    self.maze[x[0], i] |= 1
                    self.maze[x[0] - 1, i] |= 4
            else:
                for i in range(x[0], x[2]):
                    self.maze[i, x[1]] |= 8
                    self.maze[i, x[1] - 1] |= 2


    def permissibilities(self, cell):
        '''
        Check if the directions of a given cell are permissible.
        Return:
        (up, right, down, left)
        '''
        cell_value = self.maze[cell[0], cell[1]]
        return (cell_value & 1 == 0, cell_value & 2 == 0, cell_value & 4 == 0, cell_value & 8 == 0)

    def show_maze(self):

        turtle.setworldcoordinates(0, 0, self.width, self.height)

        wally = turtle.Turtle()
        wally.speed(0)
        wally.width(1.5)
        wally.hideturtle()
        turtle.tracer(0, 0)
        #print('heer')
        for i in range(self.num_rows):
            for j in range(self.num_cols):
                permissibilities = self.permissibilities(cell = (i,j))
                turtle.up()
                wally.setposition((j * self.grid_width, i * self.grid_height))
                # Set turtle heading orientation
                # 0 - east, 90 - north, 180 - west, 270 - south
                wally.setheading(0)
                if not permissibilities[0]:
                    wally.down()
                else:
                    wally.up()
                wally.pencolor('black')
                if(i == 3 and (j == 21 or j == 20)):
                    wally.pencolor('yellow')
                if(i == 0 and (j == 6 or j == 7)):
                    wally.pencolor('yellow')
                wally.forward(self.grid_width)
                
                wally.setheading(90)
                wally.up()
                if not permissibilities[1]:
                    wally.down()
                else:
                    wally.up()
                wally.pencolor('black')
                if((i == 11 or i == 12)  and j == 26):
                    wally.pencolor('yellow')

                wally.forward(self.grid_height)
                
                wally.setheading(180)
                wally.up()
                if not permissibilities[2]:
                    wally.down()
                else:
                    wally.up()
                wally.pencolor('black')
                if(i == 32 and (j == 25 or j == 26)):
                    wally.pencolor('yellow')

                if(i == 2 and (j == 21 or j == 20)):
                    wally.pencolor('yellow')

                wally.forward(self.grid_width)
                
                wally.setheading(270)
                wally.up()
                if not permissibilities[3]:
                    wally.down()
                else:
                    wally.up()
                    
                wally.pencolor('black')
                if((i == 25 or i == 26) and j == 0):
                    wally.pencolor('yellow')

                wally.forward(self.grid_height)
                
                wally.up()

        turtle.update()


    def weight_to_color(self, weight):

        return '#%02x00%02x' % (int(weight * 255), int((1 - weight) * 255))


    def show_particles(self, particles, show_frequency = 1):

        turtle.shape('tri')

        for i, particle in enumerate(particles):
            if i % show_frequency == 0:
                turtle.setposition((particle.x, particle.y))
                turtle.setheading(90 - particle.heading)
                turtle.color(self.weight_to_color(particle.weight))
                turtle.stamp()
        
        turtle.update()

    def show_estimated_location(self, particles):
        '''
        Show average weighted mean location of the particles.
        '''

        x_accum = 0
        y_accum = 0
        heading_accum = 0
        weight_accum = 0

        num_particles = len(particles)

        for particle in particles:

            weight_accum += particle.weight
            x_accum += particle.x * particle.weight
            y_accum += particle.y * particle.weight
            heading_accum += particle.heading * particle.weight

        if weight_accum == 0:

            return False

        x_estimate = x_accum / weight_accum
        y_estimate = y_accum / weight_accum
        heading_estimate = heading_accum / weight_accum

        turtle.color('orange')
        turtle.setposition(x_estimate, y_estimate)
        turtle.setheading(90 - heading_estimate)
        turtle.shape('turtle')
        turtle.stamp()
        turtle.update()

    def clear_objects(self):
        turtle.clearstamps()


class Particle(object):

    def __init__(self, x, y, maze, heading = None, weight = 1.0, sensor_limit = None, noisy = False):

        if heading is None:
            heading = np.random.uniform(0,360)
        self.x = x
        self.y = y
        self.heading = 0
        self.weight = weight
        self.maze = maze
        self.sensor_limit = sensor_limit

        if noisy:
            self.add_noise()

    @property
    def state(self):

        return (self.x, self.y, self.heading)

    def add_noise(self):
        std = max(self.maze.grid_height, self.maze.grid_width) * 0.2
        self.x = self.x + np.random.normal(0, std)
        self.y = self.y + np.random.normal(0, std)
        self.heading = self.heading + np.random.normal(0, 360 * 0.05) 

    def read_sensor(self, maze):
        readings = []
        for i in range(len(self.maze.landmarks)):
            d = math.sqrt(math.pow((self.x - self.maze.landmarks[i][0]), 2) + math.pow((self.y - self.maze.landmarks[i][1]), 2))
            d = d / 10.0            
            if(d < 10):
                readings.append(d)
        return readings
    
    def try_move(self, maze, cur_pose, prev_pose, noisy = False):
        # angle = math.radians(self.heading)        
        # x = self.x + (dx * math.cos(angle) - dy * math.sin(angle))
        # y = self.y + (dx * math.sin(angle) + dy * math.cos(angle))
        # self.x = x
        # self.y = y
        alpha = [0.05, 0.05, 0.1, 0.1]
        deltaRot1 = math.atan2(cur_pose[1] - prev_pose[1], cur_pose[0] - prev_pose[0]) - prev_pose[2]
        #print(deltaRot1)
        #print(cur_pose, prev_pose)
        deltaTrans = 10 * math.sqrt((cur_pose[0] - prev_pose[0]) ** 2 + (cur_pose[1] - prev_pose[1]) ** 2)
        print("Second:", cur_pose)
        deltaRot2 = cur_pose[2] - prev_pose[2] - deltaRot1

        trueRot1 = deltaRot1 - self.sample(mu=0, sigma=alpha[0] * math.fabs(deltaRot1) + alpha[1] * math.fabs(deltaTrans))

        trueTrans = deltaTrans - self.sample(mu=0, sigma=alpha[2] * math.fabs(deltaTrans) + alpha[3] * (
        math.fabs(deltaRot1) + math.fabs(deltaRot2)))

        trueRot2 = deltaRot2 - self.sample(mu=0, sigma=alpha[0] * math.fabs(deltaRot2) + alpha[1] * math.fabs(deltaTrans))

        x = self.x + trueTrans * math.cos(math.radians(self.heading) + trueRot1)
        y = self.y + trueTrans * math.sin(math.radians(self.heading) + trueRot1)
        theta = self.heading + math.degrees(trueRot1 + trueRot2)

        if((x >= 29 and x <= 241 and y >= 29 and y <= 301) or (x <= 1 or y <= 1 or x >= 268 or y >= 328)):
            return 0
        self.x = x
        self.y = y
        self.heading = theta
        return 1
    def sample(self, mu, sigma):
        return np.random.normal(mu, sigma)
        

class WeightedDistribution(object):

    def __init__(self, particles):
        
        accum = 0.0
        self.particles = particles
        self.distribution = list()
        for particle in self.particles:
            accum += particle.weight
            self.distribution.append(accum)

    def random_select(self):

        try:
            return self.particles[bisect.bisect_left(self.distribution, np.random.uniform(0, 1))]
        except IndexError:
            # When all particles have weights zero
            return None

def weight_gaussian_kernel(x1, x2):
    alpha = 0
    dist = 0

    if(len(x2) == 0):
        return 0
    
    x2 = np.asarray(x2)
    wt = 0
    wt2 = 0
    for i in range(len(x1)):
        wt += 1.0 / np.min(np.abs(np.subtract(x2, x1[i])))
        wt2 += np.min(np.abs(np.subtract(x2, x1[i])))
    wt2 = ((1 - alpha) / wt2) + (alpha * wt)
    return wt2