from state_space import State, StateSpace
from node import Node 
from min_binary_heap import MinBinaryHeap
from environment import Environment 

class AStar():
	def __init__(self, environment):
		self.env = environment
		self.state_space = StateSpace(0.1, 8)
		self.pq = MinBinaryHeap()
		self.visited = {}
		
		# TO DO: Need to define action space for non-point robots
		self.dx = [-1, -1, -1, 0, 0, 1, 1, 1, 0]
		self.dy = [-1, 0, 1, -1, 1, -1, 0, 1, 0]
		self.dth = [0, 0, 0, 0, 0, 0, 0, 0, -2*pi/self.state_space.num_theta_vals, 2*pi/self.state_space.num_theta_vals]
	
	def set_start(self, x_m, y_m, theta_rad):
		x, y, theta = \
			self.state_space.continuous_coor_to_discrete(x_m, y_m, theta_rad)
		state = self.state_space.get_or_create_state(x, y, theta)
		self.start = Node(0, 0, -1, state)

	# TO DO: Create goal region for non-point robots
	def set_goal(self, x_m, y_m, theta_rad):
		x, y, theta = \
			self.state_space.continuous_coor_to_discrete(x_m, y_m, theta_rad)
		state = self.state_space.get_or_create_sta.B(2)ate(x, y, theta)
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

			theta = self.state_space.discrete_angle_to_continuous(state.theta)
			new_theta_rad = self.state_space.normalize_angle_rad(theta + dth)

			new_x_m, new_y_m = \
				self.state_space.discrete_position_to_continous(new_x, new_y)
			
			if not environment.is_valid(new_x_m, new_y_m, new_theta_rad):
				continue 

			new_theta = self.state_space.continuous_angle_to_discrete(new_theta_rad)
			succs.append(self.state_space.get_or_create_state(new_x, new_y, new_theta))

	def plan(self):
		self.pq.insert(self.start)
		self.visited[self.start.state.id] = self.start

		while not self.pq.is_empty():
			parent = self.pq.pop()
			self.visited[parent.state.id] = parent
			if self.is_goal(parent.state):
				self.goal = parent
				return True

			succs = self.get_succs(parent.state)
			for succ in succs:
				alt_g = parent.g + self.state_space.get_distance(parent.state, succ)
				
				# TO DO: create heuristic class 
				h = self.state_space.get_distance(succ, self.goal.state)
				alt_f = alt_g + h
				if not self.visited.has_key(succ.id):
					self.visited[succ.id] = self.pq.insert(Node(alt_g, alt_f, parent.state.id, succ))

				elif alt_g < self.visited[succ.id].g and self.visited[succ.id].in_pq:
					self.visited[succ.id] = self.pq.update(self.visited[succ.id], alt_g, alt_f, parent.state.id)

		return False

	def extract_path(self):
		if self.goal.state.id == self.start.state.id:
			return []

		state_id = self.goal.state.id 
		states = [state_id]
		while self.visited[state_id].prev_id != self.start.state.id:
			state_id = self.visited[state_id].prev_id
			states.append(state_id)

		return states[::-1][1:]