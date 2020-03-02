from robots.robot import Robot 

class CircleRobot(Robot):
	def __init__(self, radius_m):
		self.radius_m = self.get_radius(radius_m)

	# Find the radius (m) for the collison circles
	def get_radius(self, radius_m):
		return radius_m
	
	# Given the continuous state find the centers of the collision circles
	# Return: [(x1, y1), (x2, y2),..., (xn, yn)]
	def get_collision_circles(self, x_m, y_m, theta_rad):
		return [(x_m, y_m)]