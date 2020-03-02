from robots.robot import Robot 
import math

class RectangleRobot(Robot):
	def __init__(self, length_m, width_m):
		self.length_m = length_m
		self.width_m = width_m
		self.radius_m = self.get_radius(self.length_m, self.width_m)

	# Find the radius (m) for the collison circles
	def get_radius(self, length_m, width_m):
		if self.length_m == self.width_m:
			return math.sqrt(math.pow(length_m, 2)+math.pow(width_m, 2))/2
		# TODO 
		return 0

	# Given the continuous state find the centers of the collision circles
	# Return: [(x1, y1), (x2, y2),..., (xn, yn)]
	def get_collision_circles(self, x_m, y_m, theta_rad):
		# TODO
		print("self: ", self,
			"\tx_m: ", x_m, 
			"\ty_m: ", y_m,
			"\tradius: ", theta_rad)
		center = []
		
		return [(x_m, y_m)]