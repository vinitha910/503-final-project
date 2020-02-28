from planner import AStar
from environment import Environment 
import numpy as np

if __name__ == "__main__":
	env = Environment(5, 5, 0.1)
	o1 = env.create_obstacle(0.5, 0.5, 2.0, 3.0, 0.0)
	o2 = env.create_obstacle(3, 4, 4.0, 3.0, 0.0)

	planner = AStar(env)
	planner.set_start(0.5, 0.5, 0.0)
	planner.set_goal(4.0, 4.0, 0.0)
	planner.plan()