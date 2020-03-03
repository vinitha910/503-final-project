# Adapted from https://docs.python.org/3.6/library/heapq.html#priority-queue-implementation-notes

from node import Node 
from heapq import heappush, heappop, heapify  
import itertools 
import sys 

class MinBinaryHeap:
    def __init__(self):
        self.pq = []                         # list of entries arranged in a heap
        self.entry_finder = {}               # mapping of tasks to entries
        self.REMOVED = '<removed-task>'      # placeholder for a removed task
        self.counter = itertools.count()     # unique sequence count

    def is_empty(self):
        return True if len(self.entry_finder) == 0 else False

    def insert(self, task, priority=0):
        'Add a new task or update the priority of an existing task'
        if task in self.entry_finder:
            remove(task)
        count = next(self.counter)
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heappush(self.pq, entry)

    def remove(self, task):
        'Mark an existing task as REMOVED.  Raise KeyError if not found.'
        entry = self.entry_finder.pop(task)
        entry[-1] = self.REMOVED

    def pop(self):
        'Remove and return the lowest priority task. Raise KeyError if empty.'
        while self.pq:
            priority, count, task = heappop(self.pq)
            task.in_pq = False
            if task is not self.REMOVED:
                del self.entry_finder[task]
                return task
        raise KeyError('pop from an empty priority queue')