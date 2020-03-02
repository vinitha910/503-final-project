import math
from math import cos, sin
from scipy import spatial
import numpy as np
import sys 
import itertools

class Environment:
    def __init__ (self, length, width, obstacles_params):
        self.length = length
        self.width = width
        self.obstacle_corners = []
        self.obstacles = set()
        self.all_points = set(itertools.product(range(0, width), range(0, length)))
        self.create_obstacles(obstacles_params)
        
    # Checks if x,y is in bounds of environemnt
    def is_in_bounds(self, x, y):
        if (x >= 0 and y >= 0 and x < self.width and y < self.length):
            return True
        return False

    # Finds corners of the obstacles to visualize
    def find_corners(self, corners):
        xs = [i[0] for i in corners]
        ys = [i[1] for i in corners]
        min_x = min(xs)
        min_y = min(ys)
        max_x = max(xs)
        max_y = max(ys)

        # top left, top right, bottom right, bottom left
        corners = [(min_x, max_y), (max_x, max_y), (max_x, min_y), (min_x, min_y)]
        self.obstacle_corners.append(corners)

    def create_obstacles(self, obstacles_params):
        for l, w, x, y in obstacles_params:
            self.create_obstacle(l, w, x, y)

        distance_matrix = spatial.distance_matrix(list(self.all_points), list(self.obstacles))
        self.distance_map = dict(zip(self.all_points, np.amin(distance_matrix, axis=1)))

    def create_obstacle(self, length, width, x, y):
        points = []
        for i in range(x - width, x + width + 1):
            for j in range(y - length, y + length + 1):
                if self.is_in_bounds(i, j):
                    self.obstacles.add((i, j))
                    points.append((i, j))
        self.find_corners(points)
