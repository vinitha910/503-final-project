from planner import AStar
from environment import Environment 
import numpy as np
import time 
from math import pi
from visualize import visualize 
from state_space import StateSpace

if __name__ == "__main__":
	# Input environment length, width and grid resolution (meters)
	env = Environment(5, 5)
	state_space = StateSpace(.1,8)
	# Input obstacle length (m), width (m), x (m), y (m), theta (radians)
	# Make sure all input values are floats
	o1 = env.create_obstacle(1.5, 1.0, 2.0, 2.0, pi/4)
	o2 = env.create_obstacle(2, 0.5, 2.0, 3.0, 0.0)

	planner = AStar(env, state_space)

	# Input x (m), y (m), theta (radians)
	planner.set_start(0.5, 0.5, 0.0)
	planner.set_goal(4.0, 4.0, 0.0)

	# Planner return whether or not it was successful, 
	# the number of expansions, and time taken 
	success, num_expansions, time = planner.plan()

	# If planner was successful, extract the path
	if success:
		#path = [[0,0,0], [.5,.5,0], [2.5, 1.5, 0]] 
		path = planner.extract_path()
		visualize(env, path)