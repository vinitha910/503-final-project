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

        #Robot visualization as an arrow:
        #from https://stackoverflow.com/questions/58360395/plotting-robot-path-and-orientation-using-python-matplotlib
        for i in range(len(path)):
            if ((len(path) > 1 )& (i < len(path)-1)):
                if (i == 0):
                    plt.plot(path[i][0], path[i][1], color='green', marker='o')
                j = i+1
                self.connectpoints(path, i, j)
            if (i == len(path)-1):
                plt.plot(path[i][0], path[i][1], color='red', marker='o')
                #plt.arrow(path[i][0], path[i][1], 1, 1, head_width=100, head_length=100)
        plt.show()

    def connectpoints(self, path,i,j):
        state = path[i]
        nextState = path[j]
        x1, x2 = state[0], nextState[0]
        y1, y2 = state[1], nextState[1]
        theta = state[2] #will need to be updated
        plt.plot([x1,x2],[y1,y2],'k-')
