from abc import ABC, abstractmethod

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