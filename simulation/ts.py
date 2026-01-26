# This is going to be the python file for the TS word
import numpy as np
import heapq
from enum import Enum
# node for the TS system. 

# state = placement in world

# currently ignored
class Actions(Enum):
    stay    = 0
    left    = 1
    right   = 2
    up      = 3
    down    = 4
    open    = 5
    close   = 6

# moving between cells
class Transitions(Enum):
    stay    = 0
    left    = 1
    right   = 2
    up      = 3
    down    = 4

# set of booleans in each state
class AtomicPropositions(Enum):
    goal    = 0
    unsafe  = 1
    pickup  = 2

# label: Tells you which AP are true in each state
class Labels(Enum):
    a       = 0


# LTL node (Pi, Pi0, Act, trans, AP, L)
#       Pi is the set states
#       Pi0 is the set of init states
#       Act is the action list
#       Trans in the state tranistions
#       AP is the atomic prepositions
#       L is the labels

# need to find accceptable run in T (x) A

class TransitionSystem:
    def __init__(self):
        self.S              = {all (x,y) in grid}
        self.actions        = Actions
        self.transitions    = Transitions
        self.initals        = 0
        self.AP             = AtomicPropositions
        self.labeling       = Label_function()

    def Label_function(self, World):
        for state in World:
            # init labels. 
            return
    
    


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
        self.goal   = {(5,5), (7,1), (2,10)}

    # can use the occupancy grid to make state labels
    def build_map(self):
        occupancy_grid = np.zeros(self.size, dtype=int)

        occupancy_grid[3:7, 3]  = 100
        occupancy_grid[0:5, 6]  = 100
        occupancy_grid[5, 7]    = 100
        occupancy_grid[2, 8]    = 100
        occupancy_grid[3, 9]    = 100

        return occupancy_grid

# make robot node 



# define tranistions


# make visulization function (both world and ts system)


# init node and a 