from planner import AStar, TIMEOUT
from environment import Environment 
import numpy as np
import time 
from math import pi
from visualize import Visualizer 
from state_space import StateSpace
from robots.rectangle_robot import RectangleRobot 
from robots.circle_robot import CircleRobot
from cma import CMA
import sys
import os.path
import csv

path = "run-data-"+str(int(time.time()) % 10000000)+".csv"
data_file = open(path, "w")
csv_writer = csv.writer(data_file, delimiter=',')

def clamp_obs(obs):
    return (obs-(obs-1)*(obs<1)).astype(np.int)

def run_planner(env_parameters, render=None):
        resolution_m = 0.01
        params = clamp_obs(env_parameters)

        # Statespace can take a PointRobot, SquareRobot, RectangleRobot objects
        # robot = PointRobot(0, 0)
        # robot = CircleRobot(3,3)
        robot = RectangleRobot(4,4)
        # robot = RectangleRobot(3,1)

        # Takes discrete values, divide continuous values by resolution
        # Parameters: environment length, width, 2D array with obstacle parameters
        # e.g. [[l1, w1, x1, x2], [l2, w2, x2, y2],..., [ln, wn, xn, yn]]
        env = Environment(100, 100, [params[:4], params[4:8]])

        # Parameters: resolution (m), number of theta values, robot object,
        # and environment object
        state_space = StateSpace(resolution_m, 8, robot, env)

        planner = AStar(state_space)
        error = False
        path = []
        success, num_expansions, planning_time = True, 0, 0.0

        # Input x (m), y (m)
        if not (planner.set_start(0.4, 0.7, pi/4)):
            success = False # no expansions, since initial config was invalid
        if not (planner.set_goal(0.7, 0.8, pi/4)):
            success = False # ditto

        # Planner return whether or not it was successful,
        # the number of expansions, and time taken (s)
        if success:
            try:
                success, num_expansions, planning_time = planner.plan()
                if success:
                    path = planner.extract_path()
                if planning_time >= TIMEOUT:
                    print("Planning timed out")
                    error = True
            except StandardError:
                print("Unexpected error:", sys.exc_info()[0])
                error = True

        if error or render:
            vis = Visualizer(env, state_space, robot)
            vis.visualize(path, save=True)
        else:
            print("    ", end='')

        print("Success:", success, "\tExpansions:", num_expansions, "\tTime:", planning_time)
        csv_writer.writerow([
            env_parameters,
            error,
            not not render,
            success,
            num_expansions,
            planning_time,
            ])

        if error:
            print("Your planner is buggy! Check out this failing test:", ", ".join(params.astype(str)))
            sys.exit(0)

        return -num_expansions

if __name__ == "__main__":
    initial_mean = np.array([20, 5, 57, 58, 5, 5, 44, 85])
    initial_mean = np.array([59,1,59,60,1,24,38,110])
    initial_sigma = 2.0
    initial_cov = np.eye(len(initial_mean))
    opzer = CMA(run_planner, initial_mean, initial_sigma, initial_cov)
    max_iters = 30
    min_val = 0
    convergence_patience = 5
    remaining_patience = convergence_patience
    for i in range(max_iters):
        print("Iteration", i)
        val = run_planner(opzer.mean[:,0], render=True)
        if (val < min_val):
            remaining_patience = convergence_patience
            min_val = val
        else:
            remaining_patience = remaining_patience - 1
        data_file.flush()
        opzer.iter()
    run_planner(opzer.mean[:,0], render=True)
    print("Your planner is valid! We could find no failing tests!")

