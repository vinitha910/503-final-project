
# 0 for no bug, > 0 for one of the bugs, see paper
BUG_NO = [0]
BUGNO_NONE = 0

# The planner will expand every node in the environment and then return
# unsuccesful 
BUG_INVALID_GOAL = 1

BUG_CONSTRAINED_GOAL = 2

# The planner does not update the goal to what it expanded in the goal region
# and can try to find a path to the exact goal configuration that may not have
# been visited (i.e. if a state has not been visited it cannot find a path)
BUG_INCORRECT_GOAL = 3

BUGNO_OVERFLOW = 5
BUGNO_NONTERMINATING = 6
