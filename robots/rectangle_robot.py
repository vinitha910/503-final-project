from robots.robot import Robot 
import math

class RectangleRobot(Robot):
	def __init__(self, length_m, width_m):
		self.length_m = length_m
		self.width_m = width_m
		self.radius_m = self.get_radius(self.length_m, self.width_m)

	# Find the radius (m) for the collison circles
	def get_radius(self, length_m, width_m):
		#If a square, return radius of circle that envlops square
		if self.length_m == self.width_m:
			return math.sqrt(math.pow(length_m, 2)+math.pow(width_m, 2))/2
		#If a rectangle, radius is hypotenuse of a triangle with two sides that = 1
		return math.sqrt(2)/2

	# Given the continuous state find the centers of the collision circles
	# Return: [(x1, y1), (x2, y2),..., (xn, yn)]
	def get_collision_circles(self, x_m, y_m, theta_rad):
		#If a square, return the center point
		if self.length_m == self.width_m:
			return [[(x_m), (y_m)]]

		#if a rectangle, return centers of all tiny squares in the rectangle
		centers = []
		for i in range(self.width_m):
			if (self.width_m > 1) & (i < self.width_m -1):
				x1, x2 = i, i+1
				for j in range(self.length_m):
					if (self.length_m > 1) & (j < self.length_m -1):
						y1, y2 = j, j+1
						centers += [[(x1+((x2-x1)/2)), (y1+((y2-y1)/2))]]
		return centers