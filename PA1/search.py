# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util

class Node:
    
    def __init__(self):
        self.pos = [];          # locaiton of this node
        self.parent = None;     # parent node (none indicates this is top node)
        self.direction = None;  # direction we will take on this node
        self.g_cost = 0;        # g cost (A* search)
        self.h_cost = 0;        # heuristic cost (A* search only)
        self.step_cost = 0;
        self.children = [];     # child nodes

    def f_cost(self):
        return self.g_cost + self.h_cost;
    

def reconstructPath(explored,goal):
    from game import Directions;
    
    dirs = [];
    k = goal;   # build it up in reverse
    while k is not None:
        dirs.append(k[1]);
        k = explored[k];
    dirs.reverse();
    
    return dirs;

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  
     The sequence must be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  successorsExplored={}
  explored=set()
  stack = util.Stack()
  
  for successor in problem.getSuccessors(problem.getStartState()):
    stack.push( (successor, None) )
    
  explored.add(problem.getStartState())
  if(problem.isGoalState(problem.getStartState())):
    return [];      # nothing to do

  while(stack.isEmpty() is False):
    successors = stack.pop();   # all successors of last node
    if(successors[0][0] in explored):   # already explored? break on through...
      continue
    
    # if not explored, let's look at the successor nodes...
    successorsExplored[successors[0]] = successors[1];
    explored.add(successors[0][0])  # pop in the successor node
    
    if(problem.isGoalState(successors[0][0])):
      return reconstructPath(successorsExplored, successors[0])
    
    for successor in problem.getSuccessors(successors[0][0]):
      if(successor[0] not in explored):
        stack.push((successor, successors[0]))
        
  return None

    
    
def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 74]"
  successorsExplored={}
  explored=set()
  queue = util.Queue()
  
  for successor in problem.getSuccessors(problem.getStartState()):
    queue.push((successor, None))
    
  explored.add(problem.getStartState());# since we already explore the first one
  if(problem.isGoalState(problem.getStartState())):
    return [];      # nothing to do here...

  # explore while we can still explore
  while(queue.isEmpty() is False):
    successors = queue.pop();   
    
    
    # if we've explored this already, don't do it again
    if(successors[0][0] in explored):
      continue;
  
    successorsExplored[successors[0]] = successors[1]
    explored.add(successors[0][0])

    if(problem.isGoalState(successors[0][0])):
      return reconstructPath(successorsExplored, successors[0])
    
    for successor in problem.getSuccessors(successors[0][0]):
      if(successor[0] not in explored):
        queue.push((successor, successors[0]))
        
  return None


def getParentCost( node ):
    cost = 0;
    itr = node;
    while itr.parent is not None:
        cost += itr.step_cost;
        itr = itr.parent;
    return cost;
   
              
def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  successorsExplored={}
  g_n={}
  queue = util.PriorityQueue()
  
  for successor in problem.getSuccessors(problem.getStartState()):
    queue.push((successor, None), successor[2] + heuristic(successor[0], problem))
  
  g_n[problem.getStartState()] = 0
  if(problem.isGoalState(problem.getStartState())):
    return []

  while(not queue.isEmpty()):
    successors = queue.pop()
    if(successors[0][0] in g_n):
      continue
  
    successorsExplored[successors[0]] = successors[1]
    if(successors[1] is not None):
      g_n[successors[0][0]] = g_n[successors[1][0]] + successors[0][2]
    else:
      g_n[successors[0][0]] = successors[0][2]
    
    if(problem.isGoalState(successors[0][0])):
      return reconstructPath(successorsExplored, successors[0])
    
    for successor in problem.getSuccessors(successors[0][0]):
      if(successor[0] not in g_n):
        queue.push((successor, successors[0]), g_n[successors[0][0]] + successor[2] + heuristic(successor[0], problem))
  
  return None

def uniformCostSearch(problem):
    " Okay, let's just face it, this is just A* with a null heuristic "
    " good thing the default heuristic is the null heuristic "
    " because i'm lazy. hur hur hur"
    return aStarSearch(problem);

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
