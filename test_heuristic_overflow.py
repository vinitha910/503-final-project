from planner_heu_overflow import AStar, TIMEOUT
from validate import Validation

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

if __name__ == "__main__":
    robot = CircleRobot(3)
    # planner, timeout, validator, robot type
    opt_validator = Validation(AStar, TIMEOUT, CMA, robot)
    opt_validator.validate()