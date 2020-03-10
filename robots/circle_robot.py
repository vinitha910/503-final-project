from robots.robot import Robot 
import numpy as np

class CircleRobot(Robot):
	def __init__(self, radius_m):
		self.radius_m = self.get_radius(radius_m)

	def get_radius(self, radius_m):
		return radius_m
		
	# Given the continuous state find the centers of the collision circles
	# Return: [(x1, y1), (x2, y2),..., (xn, yn)]
	def get_collision_circles(self, x_m, y_m, theta_rad):
		return [(x_m, y_m)]

	def get_perimeter_points(self, x_m, y_m, theta_rad):
		perimeter_points = []
		center = self.get_collision_circles(x_m, y_m, theta_rad)
		deltas = [-self.radius_m, self.radius_m]
		for dx in deltas:
			for dy in deltas:
				corner = np.array(center) + np.array([dx, dy])
				perimeter_points.append(corner)

		return perimeter_points