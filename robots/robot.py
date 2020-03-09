from abc import ABC, abstractmethod
from math import cos, sin 
import numpy as np 


class Robot(ABC):
    def __init__(self, length_m, width_m):
        self.length_m = length_m
        self.width_m = width_m
        self.radius_m = self.get_radius()

    @abstractmethod
    def get_radius(self):
        raise NotImplementedError

    # Given the continuous state find the centers of the collision circles
    # Return: [(x1, y1), (x2, y2),..., (xn, yn)]
    @abstractmethod
    def get_collision_circles(self, x_m, y_m, theta_rad):
        return [(x_m, y_m)]

    def rotate_point(self, point, center, theta_rad):
        trans_point = point - center
        rotated_point = np.array([cos(theta_rad)*trans_point[0] - sin(theta_rad)*trans_point[1],
                                  sin(theta_rad)*trans_point[0] + cos(theta_rad)*trans_point[1]])
        return rotated_point + center