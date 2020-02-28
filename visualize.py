from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
import matplotlib
import matplotlib.pyplot as plt
import random
import math
import numpy as np

def visualize(env, robot_x, robot_y, robot_theta):
    #Plot obstacles as Polygon patches: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.patches.Rectangle.html
    patches = []
    ax = plt.gca()
    for obstacle in env.obstacles:
        points=[]
        for i in range(len(obstacle.corners)):
            points += [[obstacle.corners[i][0], obstacle.corners[i][1]]]
        p = matplotlib.patches.Polygon(points, closed=True, fill=True)
        ax.add_patch(p)
    ax.set_xlim([0, env.width_m])
    ax.set_ylim([0, env.length_m])

    #Robot visualization as an arrow:
    #from https://stackoverflow.com/questions/58360395/plotting-robot-path-and-orientation-using-python-matplotlib
    #plt.arrow(robot_x, robot_y, cos(robot_theta), sin(robot_theta))
    plt.show()