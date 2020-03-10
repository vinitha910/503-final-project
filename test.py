from planner import AStar
from environment import Environment 
import numpy as np
import time 
from math import pi
from visualize import Visualizer 
from state_space import StateSpace
from robots.rectangle_robot import RectangleRobot 
from robots.circle_robot import CircleRobot

if __name__ == "__main__":
    resolution_m = 0.01

    # Statespace can take CircleRobot or RectangleRobot objects
    robot = RectangleRobot(0.04, 0.02) #Rectangle

    # Takes discrete values, divide continuous values by resolution
    # Parameters: environment length, width, 2D array with obstacle parameters
    # e.g. [[l1, w1, x1, x2], [l2, w2, x2, y2],..., [ln, wn, xn, yn]] 
    env = Environment(30, 30, [[6, 2, 19, 17], [2, 2, 14, 26]])

    # Parameters: resolution (m), number of theta values, robot object, 
    # and environment object 
    state_space = StateSpace(resolution_m, 8, robot, env)

    planner = AStar(state_space)

    path = []
    pts = [0.1, 0.1, 0.2, 0.25]
    # Input x (m), y (m)
    if planner.set_start(pts[0], pts[1], pi/4) and planner.set_goal(pts[2], pts[3], pi/4):

        # Planner return whether or not it was successful,
        # the number of expansions, and time taken (s)
        success, num_expansions, planning_time = planner.plan()

        if success:
            _, path = planner.extract_path()

    # Remove this when running optimization
    vis = Visualizer(env, state_space, robot)
    vis.visualize(path, start_end=pts)
