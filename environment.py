import math
from math import cos, sin
import numpy as np
import sys 

class Obstacle:
    def __init__(self, length_m, width_m, x_m, y_m, theta_rad):
        self.length_m = length_m
        self.width_m = width_m
        self.theta_rad = theta_rad
        self.x_m = x_m
        self.y_m = y_m

        # Find the corners of the un-rotated obstacle 
        self.find_corners()

        # Rotate the obstacle and find the rotated corners
        self.rotate()

    def find_corners(self):
        top_left = (self.x_m - self.width_m/2, self.y_m + self.length_m/2)
        top_right =  (self.x_m + self.width_m/2, self.y_m + self.length_m/2)
        bottom_right = (self.x_m + self.width_m/2, self.y_m - self.length_m/2)
        bottom_left = (self.x_m - self.width_m/2, self.y_m - self.length_m/2)
        self.corners = [top_left, top_right, bottom_right, bottom_left]

    def rotate(self):
        for i in range(len(self.corners)):
            x = self.corners[i][0]
            y = self.corners[i][1]
            rotated_x = self.x_m + (x - self.x_m)*cos(self.theta_rad) + (y - self.y_m)*sin(self.theta_rad)
            rotated_y = self.y_m - (x - self.x_m)*sin(self.theta_rad) + (y - self.y_m)*cos(self.theta_rad)
            self.corners[i] = (rotated_x, rotated_y) 

class Environment:
    def __init__ (self, length_m, width_m, resolution_m):
        self.length_m = length_m
        self.width_m = width_m
        self.resolution_m = resolution_m
        self.obstacles = []

    # checks if x,y is in bounds of environemnt
    def is_in_bounds(self, x_m, y_m):
        if (x_m >= 0 and y_m >= 0 and x_m <= self.width_m and y_m <= self.length_m):
            return True
        return False
  
    # checks if state is valid (i.e. not in collision and is in bounds)
    def is_valid(self, x_m, y_m, theta_rad):
        if (self.is_in_bounds(x_m, y_m) and not self.in_collision(x_m, y_m, theta_rad)):
            return True
        return False

    def create_obstacle(self, length_m, width_m, x_m, y_m, theta_rad):
        obs = Obstacle(length_m, width_m, x_m, y_m, theta_rad)
        for corner in obs.corners:
            if not self.is_in_bounds(corner[0], corner[1]):
                sys.exit('[Environment] Obstacle is out of bounds')
                return None
        self.obstacles.append(obs)
        return obs 

    def in_collision(self, x, y, theta):
        for obs in self.obstacles:
            sin_angle = sin(obs.theta_rad)
            cos_angle = cos(obs.theta_rad)
            temp = np.array([0,0])
            center = np.array([obs.x_m, obs.y_m])
            point = np.array([x, y])

            point -= center
            temp[0] = point[0] * cos_angle - point[1] * sin_angle
            temp[1] = point[0] * sin_angle + point[1] * cos_angle
            point  = temp + center;
              
            bottom_left = obs.corners[3]
            top_right = obs.corners[1]
            if (point[0] > bottom_left[0] and point[0] < top_right[0] and
                point[1] > bottom_left[1] and point[1] < top_right[1]):
                return True 
            return False
