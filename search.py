# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, step_cost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'step_cost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    from util import Stack
    start_state=problem.getStartState()
    front=Stack()
    front.push((start_state,[]))

    explored=set()

    while not front.isEmpty():
        state,actions=front.pop()
        if problem.isGoalState(state):
            return actions
        if state not in explored:
            explored.add(state)
            for successor, action, step_cost in problem.getSuccessors(state):
                new_actions=actions+[action]
                front.push((successor,new_actions))

    return []

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    from util import Queue

    q=Queue()
    q.push((problem.getStartState(),[]))  #(state,path)
    visited=set()

    while not q.isEmpty():
        curr_state,path=q.pop()

        if problem.isGoalState(curr_state):
            return path

        if curr_state not in visited:
            visited.add(curr_state)

            for successor, action, _ in problem.getSuccessors(curr_state):
                if successor not in visited:
                    q.push((successor, path+[action]))

    return []  #no soln

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    front=util.PriorityQueue() #using priority queue
    startState=problem.getStartState() #starting with the start state, empty
    front.push((startState,[],0),0)
    visited={}

    while not front.isEmpty():
        state,actions,cur_cost=front.pop()
        if state in visited and visited[state]<=cur_cost: #if visited and cost lower/equal to cur, skip
            continue
        visited[state]=cur_cost
        if problem.isGoalState(state):
            return actions

        for successor,action,step_cost in problem.getSuccessors(state):
            new_cost=cur_cost+step_cost
            new_actions=actions+[action]
            if successor not in visited or visited[successor]>new_cost:
                front.push((successor,new_actions, new_cost),new_cost)
    print("Solution Path:", actions)    
    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    from util import PriorityQueue
    start_state=problem.getStartState()
    front=PriorityQueue()
    front.push((start_state,[],0),heuristic(start_state,problem))

    explored=set()

    while not front.isEmpty():
        state,actions,cost=front.pop()#node with lowest cost+heuristic
        if problem.isGoalState(state):
            return actions
        if state not in explored:
            explored.add(state)
            for successor, action, step_cost in problem.getSuccessors(state):
                new_cost=cost+step_cost
                new_actions=actions+[action]
                front.push((successor,new_actions,new_cost),new_cost+heuristic(successor,problem))

    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch


'''
run the following commands to test the search algorithms
1. python pacman.py -l tinyMaze -p SearchAgent -a fn=depthFirstSearch
    python pacman.py -l mediumMaze -p SearchAgent -a fn=depthFirstSearch
    python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=depthFirstSearch

4. python pacman.py -l tinyMaze -p SearchAgent -a fn=astar,heuristic=nullHeuristic
    python pacman.py -l mediumMaze -p SearchAgent -a fn=astar,heuristic=nullHeuristic
    python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=nullHeuristic

    python pacman.py -l tinyMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
    python pacman.py -l mediumMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
    python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

6. python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5
'''