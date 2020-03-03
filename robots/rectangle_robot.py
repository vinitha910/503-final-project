from robots.robot import Robot 
import math
from math import cos, sin

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
		#print('radius: ', (math.sqrt(2)/2))
		return (math.sqrt(2)/2)

	# Given the continuous state find the centers of the collision circles
	# Return: [(x1, y1), (x2, y2),..., (xn, yn)]
	def get_collision_circles(self, x_m, y_m, theta_rad):
		#If a square, return the center point
		centers = [(x_m, y_m)]
		if self.length_m == self.width_m:
			return centers

		#if a rectangle, return centers of all tiny squares in the rectangle
		for x in range(self.width_m-1):
			for y in range(self.length_m-1):
				X = x*cos(theta_rad) - y*sin(theta_rad)
				X2 = (x+1)*cos(theta_rad) - (y+1)*sin(theta_rad)
				Y = x*sin(theta_rad) + y*cos(theta_rad)
				Y2 = (x+1)*sin(theta_rad) + (y+1)*cos(theta_rad)
				centers += [(X+((X2-X)/2), Y+((Y2-Y)/2))] #could be simplified to x + .5, y + .5
		return centers