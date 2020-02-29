from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
import matplotlib
import matplotlib.pyplot as plt
import random
import math
from math import cos, sin
import numpy as np

## Takes in environment variable and path
#  environment specified in environment.py, 
#   uses environment length, width, obstacles
#
#   path must be valid three dimensional array [x,y,theta]
##
class Visualizer:
    def __init__(self, env, state_space):
        self.env = env
        self.state_space = state_space
        self.obstacles = []

    def visualize(self, path):
        #Plot obstacles as Polygon patches: https://matplotlib.org/3.1.1/api/_as_gen/matplotlib.patches.Polygon.html
        patches = []
        ax = plt.gca()
        for corners in self.env.obstacle_corners:
            points=[]
            for i in range(len(corners)):
                points += [[corners[i][0], corners[i][1]]]
            p = matplotlib.patches.Polygon(points, closed=True, fill=True)
            ax.add_patch(p)
        ax.set_xlim([0, self.env.width])
        ax.set_ylim([0, self.env.length])

        robot_h_m = 2.0
        robot_w_m = 1.0
        #Robot path visualization:
        for i in range(len(path)):
            x = path[i][0]
            y = path[i][1]
            theta = path[i][2]
            if ((len(path) > 1 )& (i < len(path)-1)):
                j = i+1
                #robot = plt.Circle((path[i][0], path[i][1]), robot_w_m, color='pink')
                corner = self.getLowerCorner(x, y, robot_h_m, robot_w_m)
                robot = matplotlib.patches.Rectangle((corner[0],corner[1]), robot_w_m, robot_h_m, theta, color='pink')
                ax.add_artist(robot)
                if (i == 0):
                    plt.plot(path[i][0], path[i][1], color='green', marker='o')
                self.connectpoints(path, i, j)
            if (i == len(path)-1):
                plt.plot(path[i][0], path[i][1], color='red', marker='o')
        plt.show()

    def getLowerCorner(self, x, y, robot_h_m, robot_w_m):
        corner = [x - robot_w_m/2, y - robot_h_m/2]
        #Need to account for the theta, then will be done
        return corner

    def connectpoints(self, path,i,j):
        state = path[i]
        nextState = path[j]
        x1, x2 = state[0], nextState[0]
        y1, y2 = state[1], nextState[1]
        #theta = state[2] #will need to be updated
        plt.plot([x1,x2],[y1,y2],'k-')


