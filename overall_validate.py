from planner import AStar, TIMEOUT
from environment import Environment
import numpy as np
import time
from math import pi
from visualize import Visualizer
from state_space import StateSpace
from robots.rectangle_robot import RectangleRobot
from robots.circle_robot import CircleRobot
import sys
import os.path
import csv
from bug_config import *
from validate import cma_validate, random_validate, human_validate

import warnings
warnings.simplefilter("error")

run_id = str(int(time.time()) % 10000000)
path = "run-data-"+run_id+".csv"
data_file = open(path, "w")
csv_writer = csv.writer(data_file, delimiter=',')
img_path = "progress-"+run_id+".png"

NUM_OBSTACLES = 2
OBSTACLE_DIM = 4
M = NUM_OBSTACLES*OBSTACLE_DIM

global_vars = [0, False]
N_RUNS = 0
HUMAN_RENDER = 1

def clamp_obs(obs):
    return (obs-(obs-1)*(obs<1)).astype(np.int)

def run_planner(env_parameters, render=None):
        global_vars[N_RUNS] += 1

        resolution_m = 0.01
        obs_params = clamp_obs(env_parameters[:M])

        # Statespace can take a PointRobot, SquareRobot, RectangleRobot objects
        # robot = PointRobot(0, 0)
        # robot = CircleRobot(3)
        # robot = RectangleRobot(4,4)
        # robot = RectangleRobot(3,1)
        robot = RectangleRobot(0.04, 0.02)

        # Takes discrete values, divide continuous values by resolution
        # Parameters: environment length, width, 2D array with obstacle parameters
        # e.g. [[l1, w1, x1, y1], [l2, w2, x2, y2],..., [ln, wn, xn, yn]]
        env = Environment(100, 100, [obs_params[:4], obs_params[4:8]])

        # Parameters: resolution (m), number of theta values, robot object,
        # and environment object
        state_space = StateSpace(resolution_m, 8, robot, env)

        planner = AStar(state_space)
        error = False
        path = ([],[])
        success, num_expansions, planning_time = True, 0, 0.0

        # Input x (m), y (m)
        if len(env_parameters) > 8:
            sx, sy, gx, gy = env_parameters[8:12]
        else:
            sx, sy, gx, gy = [0.2, 0.2, 0.8, 0.8]

        if not (planner.set_start(sx, sy, pi/4)):
            success = False # no expansions, since initial config was invalid
        if not (planner.set_goal(gx, gy, pi/4)):
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
            except Exception:
                print("Unexpected error:", sys.exc_info())
                error = True

        if error or render:
            if render == True:
                filename = image_path
            else:
                filename = render
            vis = Visualizer(env, state_space, robot)
            vis.visualize(path[1], filename=filename, start_end=[sx, sy, gx, gy])
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
        data_file.flush()

        return error, success, num_expansions, planning_time

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python overall_validate.py <BUG_NO> <random || cma || human>")
        sys.exit(0)
    global_vars = [0, False]

    BUG_NO[0] = int(sys.argv[1])
    if sys.argv[2] == "random":
        validator = random_validate
    elif sys.argv[2] == "cma":
        validator = cma_validate
    else:
        global_vars[HUMAN_RENDER] = True
        validator = human_validate
    start_time = time.time()
    valid, failing_test = validator(run_planner)
    total_time = time.time() - start_time
    if valid:
        print("\nYour planner is valid! We could find no failing tests!")
    else:
        print("\nYour planner is buggy! Check out this failing test:", ", ".join(failing_test.astype(str)))
    print("Total time (s):", total_time)
    print("Validity correctly identified:", valid == (BUG_NO[0] == BUG_NONE))
    print("Num planner runs:", global_vars[N_RUNS])
    data_file.close()

