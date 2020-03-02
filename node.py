from state_space import State 

class Node:
	def __init__(self, g, prev_id, state_id, in_pq=False):
		self.in_pq = in_pq		  # Whether or not the node is in the priority queue
		self.g = g    			  # The g-value, for the node
		self.prev_id = prev_id
		self.state_id = state_id
