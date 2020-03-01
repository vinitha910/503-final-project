from planner import AStar
from environment import Environment 
import numpy as np
import time 
from math import pi
from visualize import Visualizer 
from state_space import StateSpace
from robots.point_robot import PointRobot 

if __name__ == "__main__":
	resolution_m = 0.01

	# Statespace can take a PointRobot, SquareRobot, RectangleRobot objects
	robot = PointRobot(0, 0)

	# Takes discrete values, divide continuous values by resolution
	# Parameters: environment length, width, 2D array with obstacle parameters
	# e.g. [[l1, w1, x1, x2], [l2, w2, x2, y2],..., [ln, wn, xn, yn]] 
	env = Environment(100, 100, [[20, 5, 60, 60]])

	# Parameters: resolution (m), number of theta values, robot object, 
	# and environment object 
	state_space = StateSpace(resolution_m, 8, robot, env)

	planner = AStar(state_space)

	# Input x (m), y (m), theta (radians)
	planner.set_start(0.1, 0.7, 0.0)
	planner.set_goal(0.7, 0.7, 0.0)
	
	# Planner return whether or not it was successful, 
	# the number of expansions, and time taken (s)
	success, num_expansions, planning_time = planner.plan()

	# If planner was successful, extract the path
	if success:
		path = planner.extract_path()
		# Remove this when running optimization
		vis = Visualizer(env, state_space)
		vis.visualize(path)

