from planner import AStar
from environment import Environment 
import numpy as np
import time 
from math import pi
from visualize import Visualizer 
from state_space import StateSpace

if __name__ == "__main__":
	resolution_m = 0.01

	# Parameters: resolution (m), number of theta values 
	state_space = StateSpace(resolution_m, 8)

	# Takes discrete values, divide continuous values by resolution
	# Input environment length, width 
	env = Environment(100, 100)
	
	# Takes discrete values, divide continuous values by resolution
	# Input obstacle length, width, x, y
	o1 = env.create_obstacle(20, 5, 60, 60)

	planner = AStar(env, state_space)

	# Input x (m), y (m), theta (radians)
	planner.set_start(0.1, 0.7, 0.0)
	planner.set_goal(0.7, 0.7, 0.0)
	
	# Planner return whether or not it was successful, 
	# the number of expansions, and time taken 
	success, num_expansions, time = planner.plan()
	print time
	# If planner was successful, extract the path
	if success:
		path = planner.extract_path()
		# Remove this when running optimization
		vis = Visualizer(env, state_space)
		vis.visualize(path)

