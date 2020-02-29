from state_space import State, StateSpace
from node import Node 
from min_binary_heap import MinBinaryHeap
from environment import Environment 
from math import pi 
import time, sys

class AStar():
	def __init__(self, environment, state_space):
		self.env = environment
		self.state_space = state_space
		self.pq = MinBinaryHeap()
		self.visited = {}
		
		# TO DO: Need to define action space for non-point robots
		self.dx = [-1, -1, -1, 0, 0, 1, 1, 1, 0]
		self.dy = [-1, 0, 1, -1, 1, -1, 0, 1, 0]
		self.dth = [0, 0, 0, 0, 0, 0, 0, 0, -2*pi/self.state_space.num_theta_vals, 2*pi/self.state_space.num_theta_vals]
	
	def set_start(self, x_m, y_m, theta_rad):
		x, y, theta = \
			self.state_space.continuous_coor_to_discrete(x_m, y_m, theta_rad)
		if not self.env.is_valid(x, y):
			sys.exit("[Planner] Invalid start state")
		state = self.state_space.get_or_create_state(x, y, theta)
		self.start = Node(0, 0, -1, state)
		# import IPython; IPython.embed()

	# TO DO: Create goal region for non-point robots
	def set_goal(self, x_m, y_m, theta_rad):
		x, y, theta = \
			self.state_space.continuous_coor_to_discrete(x_m, y_m, theta_rad)
		if not self.env.is_valid(x, y):
			sys.exit("[Planner] Invalid goal state")
		state = self.state_space.get_or_create_state(x, y, theta)
		self.goal = Node(0, 0, -1, state)
	
	def is_goal(self, state):
		x_m, y_m, theta_rad = \
			self.state_space.discrete_coor_to_continuous(
				state.x, state.y, state.theta)
		# TO DO: Check distance to goal is less than eps for
		# non-point robots
		return state == self.goal.state

	def get_succs(self, state):
		succs = []
		x_m, y_m, theta_rad = \
			self.state_space.discrete_coor_to_continuous(
				state.x, state.y, state.theta)
		for dx, dy, dth in zip(self.dx, self.dy, self.dth):
			new_x = state.x + dx
			new_y = state.y + dy

			# new_x_m, new_y_m = \
			# 	self.state_space.discrete_position_to_continous(new_x, new_y)
			
			if not self.env.is_valid(new_x, new_y):
				continue 

			theta = self.state_space.discrete_angle_to_continuous(state.theta)
			new_theta_rad = self.state_space.normalize_angle_rad(theta + dth)
			new_theta = self.state_space.continuous_angle_to_discrete(new_theta_rad)
			
			succs.append(self.state_space.get_or_create_state(new_x, new_y, new_theta))
		return succs

	# Returns whether a solution was found, the number of expansions, and the time taken 
	def plan(self):
		start = time.time()
		self.pq.insert(self.start)
		self.visited[self.start.state.id] = self.start
		num_expansions = 0

		while not self.pq.is_empty():
			parent = self.pq.pop()
			num_expansions += 1
			self.visited[parent.state.id] = parent
			if self.is_goal(parent.state):
				self.goal = parent
				return True, num_expansions, time.time() - start

			succs = self.get_succs(parent.state)
			for succ in succs:
				alt_g = parent.g + self.state_space.get_distance(parent.state, succ)
				
				# TO DO: create heuristic class 
				h = self.state_space.get_distance(succ, self.goal.state)
				
				# Scale g-value so that h is never greater than g
				# If h > g we will produce a sub-optimal path because it's as if
				# we are scaling h by some epsilon
				alt_f = 1000*alt_g + h

				if not succ.id in self.visited:
					self.visited[succ.id] = self.pq.insert(Node(alt_g, alt_f, parent.state.id, succ))

				elif alt_g < self.visited[succ.id].g and self.visited[succ.id].in_pq:
					self.visited[succ.id] = self.pq.update(self.visited[succ.id], alt_g, alt_f, parent.state.id)

		return False, num_expansions, time.time() - start

	def extract_path(self):
		if self.goal.state.id == self.start.state.id:
			return []

		state_id = self.goal.state.id 
		state = self.state_space.get_coord_from_state_id(state_id)
		x_m, y_m, theta_rad = \
			self.state_space.discrete_coor_to_continuous(state.x, state.y, state.theta)
		states = [[state.x, state.y, state.theta]]

		while self.visited[state_id].prev_id != -1:
			state_id = self.visited[state_id].prev_id
			state = self.state_space.get_coord_from_state_id(state_id)
			x_m, y_m, theta_rad = \
				self.state_space.discrete_coor_to_continuous(state.x, state.y, state.theta)
			states.append([state.x, state.y, state.theta])
		return states[::-1]
