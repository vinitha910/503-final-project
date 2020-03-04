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
from bug_config import *

def cma_validate(planner_fn):
    found_error = False
    failing_test = None
    def f(env_parameters, render=None):
        error, success, num_expansions, planning_time = planner_fn(env_parameters, render)
        if error:
            nonlocal found_error
            nonlocal failing_test
            found_error = True
            failing_test = env_parameters
        return -num_expansions

    initial_mean = np.array([20, 5, 57, 58, 5, 5, 44, 85, 40, 70, 70, 80])
    #initial_mean = np.array([59,1,59,60,1,24,38,110])
    initial_sigma = 2.0
    initial_cov = np.eye(len(initial_mean))
    opzer = CMA(f, initial_mean, initial_sigma, initial_cov)
    max_iters = 50
    min_val = 0
    convergence_patience = 5
    remaining_patience = convergence_patience
    i = 0
    while True:
        print("Iteration", i, "Patience", remaining_patience)
        val = f(opzer.mean[:,0], render=True)
        if found_error:
            return False, failing_test
        if (val < min_val):
            remaining_patience = convergence_patience
            min_val = val
        else:
            remaining_patience = remaining_patience - 1
            if remaining_patience == 0:
                break
        if i > max_iters:
            break
        i += 1
        opzer.iter()

    return True, None

