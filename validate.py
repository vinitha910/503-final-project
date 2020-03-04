#
from planner_good import AStar, TIMEOUT
from planner_heu_overflow import AStar as AStar_HO, TIMEOUT as TIMEOUT_HO

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

import warnings
warnings.simplefilter("error")

BUG_NO[0] = BUGNO_OVERFLOW

run_id = str(int(time.time()) % 10000000)
path = "run-data-"+run_id+".csv"
data_file = open(path, "w")
csv_writer = csv.writer(data_file, delimiter=',')
img_path = "progress-"+run_id+".png"

NUM_OBSTACLES = 2
OBSTACLE_DIM = 4
M = NUM_OBSTACLES*OBSTACLE_DIM


class Validation():
    def __init__(self, planner, time_out, validator, robot, max_iters=50, min_val=0, convergence_patience=5):
        self.initial_mean = np.array([20, 5, 57, 58, 5, 5, 44, 85, 40, 70, 70, 80])
        self.initial_sigma = 2.0
        self.initial_cov = np.eye(len(self.initial_mean))
        self.opzer = validator(self.run_planner, self.initial_mean, self.initial_sigma, self.initial_cov)
        self.max_iters, self.min_val, self.convergence_patience = max_iters, min_val, convergence_patience
        self.remaining_patience = convergence_patience
        self.planner = planner
        self.time_out = time_out
        self.robot = robot

    def validate(self):
        i = 0
        while True:
            print("Iteration", i, "Patience", self.remaining_patience)
            val = self.run_planner(self.opzer.mean[:, 0], render=True)
            data_file.flush()
            if (val < self.min_val):
                self.remaining_patience = self.convergence_patience
                self.min_val = val
            else:
                self.remaining_patience = self.remaining_patience - 1
                if self.remaining_patience == 0:
                    break
            if i > self.max_iters:
                break
            i += 1
            self.opzer.iter()
        print("Your planner is valid! We could find no failing tests!")

    def clamp_obs(self, obs):
        return (obs - (obs - 1) * (obs < 1)).astype(np.int)

    def run_planner(self, env_parameters, render=None):
        resolution_m = 0.01
        obs_params = self.clamp_obs(env_parameters[:M])

        # Takes discrete values, divide continuous values by resolution
        # Parameters: environment length, width, 2D array with obstacle parameters
        # e.g. [[l1, w1, x1, x2], [l2, w2, x2, y2],..., [ln, wn, xn, yn]]
        env = Environment(100, 100, [obs_params[:4], obs_params[4:8]])

        # Parameters: resolution (m), number of theta values, robot object,
        # and environment object
        state_space = StateSpace(resolution_m, 8, self.robot, env)

        planner = self.planner(state_space)
        error = False
        path = []
        success, num_expansions, planning_time = True, 0, 0.0

        # Input x (m), y (m)
        if not (planner.set_start(env_parameters[-4] / 100., env_parameters[-3] / 100., pi / 4)):
            print(env_parameters[-4], ' ', env_parameters[-5])
            success = False  # no expansions, since initial config was invalid
        if not (planner.set_goal(env_parameters[-2] / 100., env_parameters[-1] / 100., pi / 4)):
            success = False  # ditto

        # Planner return whether or not it was successful,
        # the number of expansions, and time taken (s)
        if success:
            try:
                success, num_expansions, planning_time = planner.plan()
                if success:
                    path = planner.extract_path()
                if planning_time >= self.time_out:
                    print("Planning timed out")
                    error = True
            except Exception or RuntimeWarning:
                print("Unexpected error:", sys.exc_info())
                error = True

        if error or render:
            vis = Visualizer(env, state_space, self.robot)
            vis.visualize(path, filename=img_path)
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
            print("Your planner is buggy! Check out this failing test:", ", ".join(obs_params.astype(str)),
                  ", ".join(env_parameters[M:].astype(str)))
            sys.exit(0)

        return -num_expansions


if __name__ == "__main__":
    robot = CircleRobot(3)
    # planner, timeout, validator, robot type
    opt_validator = Validation(AStar_HO, TIMEOUT_HO, CMA, robot)
    opt_validator.validate()
