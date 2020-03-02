from robots.robot import Robot 
from robots.rectangle_robot import RectangleRobot 
from matplotlib.patches import Rectangle
import matplotlib
import matplotlib.pyplot as plt
import math
import sys 


class SquareRobot(Robot):
	def __init__(self, length_m, width_m):
		self.length_m = length_m
		self.width_m = width_m
		self.radius_m = self.get_radius()

	# Find the radius (m) for the collison circles
	def get_radius(self):
		if not self.is_square():
			sys.exit('[Square Robot] Length does not equal width, instantiate as Rect Robot')
		return math.sqrt(math.pow(self.length_m, 2)+math.pow(self.width_m, 2))/2

	def is_square(self):
		if self.length_m == self.width_m:
			return True
		else:
			return False

	# Given the continuous state find the centers of the collision circles
	# Return: [(x1, y1), (x2, y2),..., (xn, yn)]
	def get_collision_circles(self, x_m, y_m, theta_rad):
		# TODO
		return [(x_m, y_m)]

	def draw(self, corner, center, theta):
		return matplotlib.patches.Rectangle((corner[0],corner[1]), self.width_m, self.length_m, angle=theta, color='pink')