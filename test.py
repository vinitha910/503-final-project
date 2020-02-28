from planner import AStar
from environment import Environment 
import numpy as np
import time 

if __name__ == "__main__":
	# Input environment length, width and grid resolution (meters)
	env = Environment(40, 40, 0.05)

	# Input obstacle length (m), width (m), x (m), y (m), theta (radians)
	# Make sure all input values are floats
	o1 = env.create_obstacle(30.0, 10.0, 20.0, 20.0, 0.0)
	o2 = env.create_obstacle(10.0, 20.0, 10.0, 30.0, 0.0)
	o3 = env.create_obstacle(10.0, 20.0, 10.0, 30.0, 0.0)

	planner = AStar(env)

	# Input x (m), y (m), theta (radians)
	planner.set_start(0.5, 0.5, 0.0)
	planner.set_goal(35.0, 35.0, 0.0)

	# Planner return whether or not it was successful, 
	# the number of expansions, and time taken 
	success, num_expansions, time = planner.plan()

	# If planner was successful, extract the path
	if success:
		path = planner.extract_path()