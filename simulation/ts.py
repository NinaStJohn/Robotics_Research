# This is going to be the python file for the TS word
import numpy as np
import heapq
from enum import Enum
# node for the TS system. 

class Actions(Enum):
    open        = 0
    close       = 1


# LTL node (Pi, Pi0, Act, trans, AP, L)
#       Pi is the set states
#       Pi0 is the set of init states
#       Act is the action list
#       Trans in the state tranistions
#       AP is the atomic prepositions
#       L is the labels

# need to find accceptable run in T (x) A

class Node:
    def __init__(self, position, parent=None):
        self.position   = position   # x,y tuple
        self.parent     = parent
        self.g          = 0             # cost start -> node
        self.h          = 0             # h from self-goal
        self.f          = 0             # total cost (g+h)
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

# super init the grid word
class World:
    def __init__(self):
        self.size   = (10,10)
        self.robot  = [0,0]
        self.goal   = {[5,5], [7,1]}

    def build_map(self):
        occupancy_grid = np.zeros(self.map_size, dtype=int)



# make robot node 



# define tranistions


# make visulization function (both world and ts system)


# init node and a 