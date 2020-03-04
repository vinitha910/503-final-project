from state_space import State, StateSpace
from node import Node 
from min_binary_heap import MinBinaryHeap
from environment import Environment 
from math import pi 
import time, sys
import numpy as np
from bug_config import *

TIMEOUT = 60.0

class AStar():
    def __init__(self, state_space):
        self.state_space = state_space
        self.pq = MinBinaryHeap()
        self.visited = {}
        
        # TO DO: Need to define action space for non-point robots
        self.dx = [-1, -1, -1, 0, 0, 1, 1, 1, 0, 0]
        self.dy = [-1, 0, 1, -1, 1, -1, 0, 1, 0, 0]
        self.dth = [0, 0, 0, 0, 0, 0, 0, 0, -2*pi/self.state_space.num_theta_vals, 2*pi/self.state_space.num_theta_vals]

    def set_start(self, x_m, y_m, theta_rad):
        x, y, theta = \
            self.state_space.continuous_coor_to_discrete(x_m, y_m, theta_rad)
        if not self.state_space.is_valid(x, y, theta):
            print("[Planner] Invalid start state")
            return False
        state = self.state_space.get_or_create_state(x, y, theta)
        self.start = Node(np.int32(0), -1, state.id)
        return True

    # TO DO: Create goal region for non-point robots
    def set_goal(self, x_m, y_m, theta_rad):
        x, y, theta = \
            self.state_space.continuous_coor_to_discrete(x_m, y_m, theta_rad)
        if not self.state_space.is_valid(x, y, theta):
            print("[Planner] Invalid goal state")
            return False
        self.goal_state = self.state_space.get_or_create_state(x, y, theta)
        self.goal = Node(np.int32(0), -1, self.goal_state.id)
        return True
            
    def is_goal(self, state):
        return state.x == self.goal_state.x and state.y == self.goal_state.y
        # TO DO: Check distance to goal is less than eps for
        # non-point robots

    def get_succs(self, state):
        succs = []
        for dx, dy, dth in zip(self.dx, self.dy, self.dth):
            new_x = state.x + dx
            new_y = state.y + dy

            theta = self.state_space.discrete_angle_to_continuous(state.theta)
            new_theta_rad = self.state_space.normalize_angle_rad(theta + dth)
            new_theta = self.state_space.continuous_angle_to_discrete(new_theta_rad)
            
            if not self.state_space.is_valid(new_x, new_y, new_theta):
                continue 

            succs.append(self.state_space.get_or_create_state(new_x, new_y, new_theta))
        return succs

    # Returns whether a solution was found, the number of expansions, and the time taken 
    def plan(self):
        start = time.time()
        self.pq.insert(self.start)
        self.start.in_pq = True
        self.visited[self.start.state_id] = self.start
        num_expansions = 0

        while not self.pq.is_empty():
            if time.time() - start > TIMEOUT: 
                break
            parent = self.pq.pop()
            assert(parent.in_pq == False)
            self.visited[parent.state_id] = parent

            num_expansions += 1

            parent_state = self.state_space.get_coord_from_state_id(parent.state_id)
            if self.is_goal(parent_state):
                self.goal = parent
                return True, num_expansions, time.time() - start

            succs = self.get_succs(parent_state)
            for succ in succs:        
                alt_g = parent.g + self.state_space.get_distance(parent_state, succ)
                
                # TO DO: create heuristic class 
                h = self.state_space.get_distance(succ, self.goal_state)
                
                # Scale g-value so that h is never greater than g
                # If h > g we will produce a sub-optimal path because it's as if
                # we are scaling h by some epsilon
                if BUG_NO[0] == BUGNO_OVERFLOW:
                    alt_f = np.int32(alt_g + h)
                else:
                    alt_f = 1000*alt_g + h

                # If the successor has not been visited OR the node is in the 
                # open list AND alt_g < the previously calculated g
                if not succ.id in self.visited or (self.visited[succ.id].in_pq 
                   and alt_g < self.visited[succ.id].g):
                    succ_node = Node(alt_g, parent.state_id, succ.id, True)
                    self.pq.insert(succ_node, alt_f)
                    self.visited[succ.id] = succ_node

                    assert(self.visited[succ.id].state_id == succ.id)

        return False, num_expansions, time.time() - start

    def extract_path(self):
        if self.goal.state_id == self.start.state_id:
            return []

        state_id = self.goal.state_id 
        state = self.state_space.get_coord_from_state_id(state_id)
        theta_rad = \
            self.state_space.discrete_angle_to_continuous(state.theta)
        states = [[state.x, state.y, theta_rad]]

        if not state_id in self.visited:
            # just draw a line from start to finish
            state_id = self.start.state_id
            state = self.state_space.get_coord_from_state_id(state_id)
            states.append([state.x, state.y, theta_rad])
            return states[::-1]

        while self.visited[state_id].prev_id != -1:
            state_id = self.visited[state_id].prev_id
            state = self.state_space.get_coord_from_state_id(state_id)
            theta_rad = \
                self.state_space.discrete_angle_to_continuous(state.theta)
            states.append([state.x, state.y, theta_rad])
        return states[::-1]
