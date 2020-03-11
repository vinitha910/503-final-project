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

NUM_OBSTACLES = 2
OBSTACLE_DIM = 4
M = NUM_OBSTACLES*OBSTACLE_DIM

global_vars = [0, False]
save_vars = [None, None, None]
N_RUNS = 0
HUMAN_RENDER = 1
DATA_FILE = 0
IMG_PATH = 1
CSV_WRITER = 2
# use oracle  1     2      3     4      5     6
ORACLE_L = [False, True, False, False, True, True]

def clamp_obs(obs):
    o = obs.copy()
    o[o < 1] = 1
    o[o > 29] = 29
    return o.astype(np.int)

def run_planner(env_parameters, render=None):
    global_vars[N_RUNS] += 1

    resolution_m = 0.01
    if not ORACLE_L[1]:
        obs_params = clamp_obs(env_parameters[:M])
        with open("temp-data-params.txt", "w") as f:
            f.write(",".join(obs_params.astype(str)))

    # Statespace can take a PointRobot, SquareRobot, RectangleRobot objects
    # robot = PointRobot(0, 0)
    # robot = CircleRobot(3)
    # robot = RectangleRobot(4,4)
    # robot = RectangleRobot(3,1)
    robot = RectangleRobot(0.04, 0.02)

    # Takes discrete values, divide continuous values by resolution
    # Parameters: environment length, width, 2D array with obstacle parameters
    # e.g. [[l1, w1, x1, y1], [l2, w2, x2, y2],..., [ln, wn, xn, yn]]
    if ORACLE_L[1]:
        env = Environment(30, 30, [])
    else:
        env = Environment(30, 30, [obs_params[:4], obs_params[4:8]])

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
        sx, sy, gx, gy = [0.05, 0.05, 0.25, 0.25]

    # Optimizing orientation
    if len(env_parameters) > 12:
        so, go = env_parameters[12], env_parameters[13]
    else:
        so, go = pi/4, pi/4

    if not (planner.set_start(sx, sy, so)):
        success = False # no expansions, since initial config was invalid
    if not (planner.set_goal(gx, gy, go)):
        success = False # ditto

    # Planner return whether or not it was successful,
    # the number of expansions, and time taken (s)
    if success:
        try:
            success, num_expansions, planning_time = planner.plan()
            if success:
                path = planner.extract_path()

                # BUG 3 -- Oracle
                if len(path[0]) == 0 and ORACLE_L[2]:
                    print("ERROR: Incorrect goal")
                    error = True
                # BUG 4 -- Oracle
                elif (not planner.check_consistency(path[0])) and ORACLE_L[3]:
                    print("ERROR: Path not consistence")
                    error = True

            else:
                # BUG 2 -- Oracle
                if ORACLE_L[1]:
                    print('ERROR: Constrained goal')
                    error = True
                # BUG 1 -- Oracle
                if len(planner.visited) >= env.get_max_expansions() and ORACLE_L[0]:
                    print("ERROR: Expanded every node in the environment")
                    error = True

            # BUG 6 -- Oracle
            if planning_time >= TIMEOUT:
                print("Planning timed out")
                if ORACLE_L[5]:
                    error = True

        # BUG 5 -- Oracle
        except Exception:
            print("Unexpected error:", sys.exc_info())
            error = True

    if error or render:
        filename = None
        if error or (render == True):
            filename = save_vars[IMG_PATH]
        elif render is not None:
            filename = render
        vis = Visualizer(env, state_space, robot)
        vis.visualize(path[1], filename=filename, start_end=[sx, sy, gx, gy])
    else:
        print("    ", end='')

    print("Success:", success, "\tExpansions:", num_expansions, "\tTime:", planning_time)
    save_vars[CSV_WRITER].writerow([
        env_parameters,
        error,
        not not render,
        success,
        num_expansions,
        planning_time,
        ])
    save_vars[DATA_FILE].flush()

    return error, success, num_expansions, planning_time

def run_seed(validator_name, prefix):
    global_vars[N_RUNS] = 0
    global_vars[HUMAN_RENDER] = False

    run_id = prefix + "-" + str(int(time.time()) % 10000000)
    path = "temp-data-"+run_id+".csv"
    save_vars[DATA_FILE] = open(path, "w")
    save_vars[CSV_WRITER] = csv.writer(save_vars[DATA_FILE], delimiter=',')
    save_vars[IMG_PATH] = "temp-data-"+run_id+".png"

    if validator_name == "random":
        validator = random_validate
    elif validator_name == "cma":
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

    n_runs = global_vars[N_RUNS]
    correct = valid == (BUG_NO[0] == BUG_NONE)
    save_vars[DATA_FILE].close()

    print("Total time (s):", total_time)
    print("Validity correctly identified:", correct)
    print("Num planner runs:", n_runs)

    return run_id, total_time, correct, n_runs

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python overall_validate.py <BUG_NO> <random || cma || human>")
        sys.exit(0)

    bugnum = int(sys.argv[1])
    validator_name = sys.argv[2]
    if bugnum < 0:
        bugnum = np.random.choice([0,5,6]) # TODO add more once these are ready
        num_trials = 1
        prefix = "bugRANDOM"+validator_name
    else:
        num_trials = 5
        prefix = "bug"+str(bugnum)+validator_name
    BUG_NO[0] = bugnum

    results = np.zeros((num_trials, 3))
    with open(prefix+".csv", "w") as f:
        for i in range(num_trials):
            result = run_seed(validator_name, prefix)
            f.write(",".join(np.array(result)) + "\n")
            results[i] = np.array(result[1:])
    print("\nStatistics for "+prefix+", bug "+str(bugnum)+" (total time in seconds, correct, number of planner runs):")
    print("Mean:", results.mean(0))
    print("Std:", results.std(0))


