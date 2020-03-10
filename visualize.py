from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
import matplotlib
import matplotlib.transforms as transforms
import matplotlib.pyplot as plt
import random
import math
from math import cos, sin, floor
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

    def visualize(self, path, filename=None, start_end=None):
        patches = []
        fig = plt.figure(figsize=(8,8))
        ax = fig.gca()
        for corners in self.env.obstacle_corners:
            points=[]
            for i in range(len(corners)):
                points += [[corners[i][0], corners[i][1]]]
            p = matplotlib.patches.Polygon(points, closed=True, fill=True, alpha=0.2)
            ax.add_patch(p)
        ax.set_xlim([0, self.env.width])
        ax.set_ylim([0, self.env.length])

        #Discretize length, width, and radius
        length = floor(self.robot.length_m/self.state_space.resolution)
        width = floor(self.robot.width_m/self.state_space.resolution)
        radius = floor(self.robot.radius_m/self.state_space.resolution)

        #Robot path visualization:
        #For every step i, which has an x,y,theta coordinate, plot robot
        for i in range(len(path)):
            center_x = path[i][0]
            center_y = path[i][1]
            theta = path[i][2]
            #If there is more than one point (path length > 0), draw path
            if len(path) > 1:
                #If this is a square/rectangluar robot, draw rectangle
                if hasattr(self.robot, 'length_m'):
                    corner = self.getLowerCorner(center_x, center_y, length, width, theta)
                    robotDraw = matplotlib.patches.Rectangle((corner[0],corner[1]), width, length, 
                        angle=math.degrees(theta), fill=False, edgecolor='pink')
                else:
                    #Draw a circle (or point, circle with radius = 0.0)
                    robotDraw = plt.Circle((center_x, center_y), radius, fill=False, edgecolor='pink')
                ax.add_artist(robotDraw)

                #If at the very beginning, add green dot
                if (i == 0):
                    plt.plot(center_x, center_y, color='green', marker='o', alpha=.8)
                #If not on the very last one, draw the lines
                if i < len(path)-1:
                    j = i+1
                    self.connectpoints(path, i, j)
            #If on last one, plot a red point
            if i == len(path)-1:
                plt.plot(center_x, center_y, color='blue', marker='o', alpha=.8)

        if start_end is not None:
            sx, sy, gx, gy = (np.array(start_end)*100).astype(int)
            plt.plot(sx, sy, color='green', marker='o', alpha=0.8)
            plt.plot(gx, gy, color='blue', marker='o', alpha=0.8)

        if filename:
            plt.savefig(filename)
        else:
            plt.show()
        plt.clf()
        plt.close()

    def getLowerCorner(self, center_x, center_y, robot_h_m, robot_w_m, theta_rad):
        
        center = np.array([center_x, center_y])
        corner =  center + np.array([-robot_w_m/2, -robot_h_m/2])
        rotated_corner = self.robot.rotate_point(corner, center, theta_rad)
        
        return rotated_corner

    def connectpoints(self, path,i,j):
        state = path[i]
        nextState = path[j]
        x1, x2 = state[0], nextState[0]
        y1, y2 = state[1], nextState[1]
        plt.plot([x1,x2],[y1,y2],'k-')


