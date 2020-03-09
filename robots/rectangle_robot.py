from robots.robot import Robot 
import math
from math import cos, sin
import numpy as np

class RectangleRobot(Robot):
    def __init__(self, length_m, width_m):
        self.length_m = length_m
        self.width_m = width_m
        self.radius_m = self.get_radius()

    # Find the radius (m) for the collison circles
    def get_radius(self):
        return (math.sqrt(((self.width_m/2)**2)*2) + 0.3*self.width_m)

    # Given the continuous state find the centers of the collision circles
    # Return: [(x1, y1), (x2, y2),..., (xn, yn)]
    def get_collision_circles(self, x_m, y_m, theta_rad):
        #If a square, return the center point
        if self.length_m == self.width_m:
            print('square')
            return [(x_m, y_m)]

        if (self.length_m / self.width_m != 2):
            sys.error('RectangleRobot length should be twice the width')
    
        center = np.array([x_m, y_m])
        rotated_midpoint = self.rotate_point(np.array([x_m, y_m + self.length_m/2.0]), center, theta_rad)
        collision_sphere_1 = (rotated_midpoint + center)/2.0

        rotated_midpoint = self.rotate_point(np.array([x_m, y_m - self.length_m/2.0]), center, theta_rad)
        collision_sphere_2 = (rotated_midpoint + center)/2.0

        centers = [tuple(collision_sphere_1), tuple(collision_sphere_2)]
        
        assert(len(centers) == self.length_m/self.width_m)
        return centers

    def get_perimeter_points(self, x_m, y_m, theta_rad):
        perimeter_points = []
        centers = self.get_collision_circles(x_m, y_m, theta_rad)
        deltas = [-self.width_m/2.0, self.width_m/2.0]
        for center in centers:
            for dx in deltas:
                for dy in deltas:
                    corner = np.array(center) + np.array([dx, dy])
                    rotated_corner = self.rotate_point(corner, np.array(center), theta_rad)
                    perimeter_points.append(rotated_corner)

        if len(perimeter_points) == 4:
            return perimeter_points

        xs = [i[0] for i in perimeter_points]
        ys = [i[1] for i in perimeter_points]
        min_x = min(xs)
        min_y = min(ys)
        max_x = max(xs)
        max_y = max(ys)

        # top left, top right, bottom right, bottom left
        perimeter_points = [(min_x, max_y), (max_x, max_y), (max_x, min_y), (min_x, min_y)]

        return perimeter_points