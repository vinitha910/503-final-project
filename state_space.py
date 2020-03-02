#!/usr/bin/env python

from math import floor, pi
import numpy as np
from robots.robot import Robot 

class State(object):
    def __init__(self, state_id, x, y, theta):
        self.id = state_id
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.theta == other.theta
    
    def __repr__(self):
        return str(self.id) + ': (' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.theta) + ')'


class StateSpace(object):
    def __init__(self, resolution, num_theta_vals, robot, env):
        self.resolution = resolution # in meters
        self.num_theta_vals = num_theta_vals
        self.states = []
        self.state_to_id_map = {}
        self.robot = robot
        self.env = env 

    def normalize_angle_rad(self, theta_rad):
        normalized_theta_rad = theta_rad

        # Get the normalized angle in the range of [-2pi, 2pi] if necessary
        if abs(normalized_theta_rad) > 2.0*pi:
            normalized_theta_rad = normalized_theta_rad - \
                int(normalized_theta_rad / (2.0*pi)) * 2 * pi

        # Get normalized angle in the range of [0, 2pi] if necessary
        if theta_rad < 0:
            normalized_theta_rad += 2 * pi

        assert(normalized_theta_rad >= 0 or normalized_theta_rad <= 2*pi)

        return normalized_theta_rad

    def discrete_angle_to_continuous(self, theta):
        return theta * (2.0*pi / self.num_theta_vals)

    def continuous_angle_to_discrete(self, theta_rad):
        bin_size = 2.0 * pi / self.num_theta_vals
        normalized_theta_rad = \
            self.normalize_angle_rad(theta_rad + bin_size/2.0) / (2*pi) * self.num_theta_vals
        return int(normalized_theta_rad)

    def get_state_id(self, x, y, theta):
        if (x, y, theta) in self.state_to_id_map:
            return self.state_to_id_map[(x, y, theta)]
        return None

    def get_coord_from_state_id(self, state_id):
        assert(state_id < len(self.states))
        return self.states[state_id]

    def create_new_state(self, x, y, theta):
        state_id = len(self.states)
        self.states.append(State(state_id, x, y, theta))
        self.state_to_id_map[(x, y, theta)] = state_id
        return self.states[state_id]

    def get_or_create_state(self, x, y, theta):
        state_id = self.get_state_id(x, y, theta)
        if state_id != None:
            return self.states[state_id]
        return self.create_new_state(x, y, theta)

    def continous_position_to_discrete(self, x_m, y_m):
        return floor(x_m/self.resolution), floor(y_m/self.resolution)

    def discrete_position_to_continous(self, x, y):
        return x * self.resolution + self.resolution/2, y * self.resolution + self.resolution/2

    def discrete_coor_to_continuous(self, x, y, theta):
        x_m, y_m = self.discrete_position_to_continous(x, y)
        theta_rad = self.discrete_angle_to_continuous(theta)
        return x_m, y_m, theta_rad

    def continuous_coor_to_discrete(self, x_m, y_m, theta_rad):
        x, y = self.continous_position_to_discrete(x_m, y_m)
        theta = self.continuous_angle_to_discrete(theta_rad)
        return x, y, theta

    def get_distance(self, state_1, state_2):
        x1, y1, th1 = self.discrete_coor_to_continuous(
            state_1.x, state_1.y, state_1.theta)
        x2, y2, th2 = self.discrete_coor_to_continuous(
            state_2.x, state_2.y, state_2.theta)

        return np.linalg.norm(
            [x1 - x2, y1 - y2, th1 - th2]) 

    def in_collision(self, x, y, theta):
        x_m, y_m, theta_rad = \
            self.discrete_coor_to_continuous(x, y, theta)

        collision_circles = self.robot.get_collision_circles(x_m, y_m, theta_rad)
        for x_m, y_m in collision_circles:
            x, y = self.continous_position_to_discrete(x_m, y_m)
            if self.env.distance_map[(x, y)] <= self.robot.radius_m:
                return True
        return False

    # Checks if state is valid (i.e. not in collision and is in bounds)
    def is_valid(self, x, y, theta):
        if (self.env.is_in_bounds(x, y) and not self.in_collision(x, y, theta)):
            return True
        return False