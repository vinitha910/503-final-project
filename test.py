from planner import AStar
from environment import Environment 
import numpy as np
import time 
from math import pi
from visualize import Visualizer 
from state_space import StateSpace
from robots.point_robot import PointRobot 
from robots.square_robot import SquareRobot 
from robots.rectangle_robot import RectangleRobot 
from robots.circle_robot import CircleRobot

if __name__ == "__main__":
	resolution_m = 0.01

	# Statespace can take a PointRobot, SquareRobot, RectangleRobot objects
	# robot = PointRobot(0, 0)
	# robot = CircleRobot(3,3)
	robot = SquareRobot(4,4)
	# robot = RectangleRobot(3,1)

	# Takes discrete values, divide continuous values by resolution
	# Parameters: environment length, width, 2D array with obstacle parameters
	# e.g. [[l1, w1, x1, x2], [l2, w2, x2, y2],..., [ln, wn, xn, yn]] 
	env = Environment(100, 100, [[20, 5, 57, 58], [5, 5, 44, 85]])

	# Parameters: resolution (m), number of theta values, robot object, 
	# and environment object 
	state_space = StateSpace(resolution_m, 8, robot, env)

	planner = AStar(state_space)

	# Input x (m), y (m)
	planner.set_start(0.4, 0.7, pi/4)
	planner.set_goal(0.7, 0.8, pi/4)
	
	# Planner return whether or not it was successful, 
	# the number of expansions, and time taken (s)
	success, num_expansions, planning_time = planner.plan()

	# If planner was successful, extract the path
	if success:
		path = planner.extract_path()
		# Remove this when running optimization
		vis = Visualizer(env, state_space, robot)
		vis.visualize(path)
