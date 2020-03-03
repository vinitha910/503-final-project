from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
import matplotlib
import matplotlib.transforms as transforms
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
    def __init__(self, env, state_space, robot):
        self.env = env
        self.state_space = state_space
        self.obstacles = []
        self.robot = robot

    def visualize(self, path, filename=None):
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

        #Robot path visualization:
        for i in range(len(path)):
            theta = path[i][2]
            center = [path[i][0], path[i][1]]
            if ((len(path) > 1 )& (i < len(path)-1)):
                j = i+1
                if hasattr(self.robot, 'length_m'):
                    corner = self.getLowerCorner(center[0], center[1], self.robot.length_m, self.robot.width_m, theta)
                    #print('x: ', x, ' y: ', y, ' theta: ', theta)
                    robotDraw = matplotlib.patches.Rectangle((corner[0],corner[1]), self.robot.width_m, self.robot.length_m, angle=math.degrees(theta), color='pink', alpha = .5)
                else:
                    center = [path[i][0], path[i][1]]
                    robotDraw = plt.Circle((center[0], center[1]), self.robot.radius_m, fill=False, color='pink', alpha=.9)
                ax.add_artist(robotDraw)
                if (i == 0):
                    plt.plot(path[i][0], path[i][1], color='green', marker='o')
                self.connectpoints(path, i, j)
            if (i == len(path)-1):
                plt.plot(center[0], center[1], color='red', marker='o', alpha=.8)
        if filename:
            plt.savefig(filename)
        else:
            plt.show()
        plt.clf()

    def getLowerCorner(self, center_x, center_y, robot_h_m, robot_w_m, theta_rad):
        #rotated_x = center_x + (corner_x - center_x)*cos(theta) + (corner_y - center_y)*sin(theta)
        #rotated_y = center_y - (corner_x - center_x)*sin(self.theta_rad) + (corner_y - center_y)*cos(self.theta_rad)
        # hyp = math.sqrt(math.pow(abs(center_x)-(robot_w_m/2),2) + math.pow(abs(center_x)-(robot_h_m/2),2))
        corner = [center_x-robot_w_m/2, center_y-robot_h_m]
        # rotated_x = corner[0]
        # rotated_y = corner[1]

        #rotated_x = center_x - (corner[0] - center_x)*cos(theta_rad) + (corner[1] - center_y)*sin(theta_rad)
        rotated_x = center_x + (robot_w_m)*cos(theta_rad) + (corner[1] - center_y)*sin(theta_rad)
        #rotated_y = center_y - (corner[0] - center_x)*sin(theta_rad) + (corner[1] - center_y)*cos(theta_rad)
        rotated_y = center_y  + (corner[1] - center_y)*cos(theta_rad)
        rotated_corner = [rotated_x , rotated_y]
        return rotated_corner

    def connectpoints(self, path,i,j):
        state = path[i]
        nextState = path[j]
        x1, x2 = state[0], nextState[0]
        y1, y2 = state[1], nextState[1]
        plt.plot([x1,x2],[y1,y2],'k-')


