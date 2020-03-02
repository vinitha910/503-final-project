from robots.robot import Robot 
import matplotlib
import matplotlib.pyplot as plt
import sys

class PointRobot(Robot):
	def __init__(self, length_m, width_m):
		self.length_m = length_m
		self.width_m = width_m
		self.radius_m = self.get_radius()

	# Find the radius (m) for the collison circles
	def get_radius(self):
		if not self.is_point():
			sys.exit('[Point Robot] Not a point, instantiate as circle')
		return 0

	def is_point(self):
		if (self.length_m == self.width_m) & (self.length_m == 0):
			return True
		else:
			return False

	# Given the continuous state find the centers of the collision circles
	# Return: [(x1, y1), (x2, y2),..., (xn, yn)]
	def get_collision_circles(self, x_m, y_m, theta_rad):
		return [(x_m, y_m)]

	def draw(self, corner, center, theta):
		print(self.radius_m)
		return plt.Circle((center[0], center[1]), self.radius_m, color='pink')

