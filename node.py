from state_space import State 

class Node:
	def __init__(self, g, f, prev_id, state):
		self.pq_id = -1           # The nodes index/priority in the priority queue
		self.in_pq = False		  # Whether or not the node is in the priority queue
		self.g = g    			  # The g-value for the node
		self.f = f
		self.prev_id = prev_id
		self.state = state

	def parent(self, idx):
		return (idx - 1)/2

	def left(self, idx):
		return 2*idx + 1

	def right(self, idx):
		return 2*idx + 2