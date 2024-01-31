# Andriani Perez 1/24/2024
# CSCE 580
# Homework 1

from typing import List, Tuple, Dict, Optional, cast
from environments.environment_abstract import Environment, State
from environments.farm_grid_world import FarmState
from environments.n_puzzle import NPuzzleState
from heapq import heappush, heappop, heapify
from collections import deque
import time
import numpy as np
import pdb


class Node:
    def __init__(self, state: State, path_cost: float, parent_action: Optional[int], parent):
        self.state: State = state
        self.parent: Optional[Node] = parent
        self.path_cost: float = path_cost
        self.parent_action: Optional[int] = parent_action

    def __hash__(self):
        return self.state.__hash__()
    #CHANGING THIS NAMING TO LT FROM GT SUDDENLY MAKES ThE CODE SO MUCH QUICKER XOR < to > XOR FLIPPING THE VARIABLES I HAVE NO IDEA WHY 
    def __gt__(self, other):
        return  self.path_cost < other.path_cost 

    def __eq__(self, other):
        return self.state == other.state



def get_next_state_and_transition_cost(env: Environment, state: State, action: int) -> Tuple[State, float]:
    """

    :param env: Environment
    :param state: State
    :param action: Action
    :return: the next state and the transition cost
    """
    rw, states_a, _ = env.state_action_dynamics(state, action)
    state: State = states_a[0]
    transition_cost: float = -rw

    return state, transition_cost



def visualize_bfs(viz, closed_states: List[State], queue: List[Node], wait: float):
    """

    :param viz: visualizer
    :param closed_states: states in CLOSED
    :param queue: states in priority queue
    :param wait: number of seconds to wait after displaying
    :return: None
    """

    if viz is None:
        return

    grid_dim_x, grid_dim_y = viz.env.grid_shape
    for pos_i in range(grid_dim_x):
        for pos_j in range(grid_dim_y):
            viz.board.itemconfigure(viz.grid_squares[pos_i][pos_j], fill="white")

    for state_u in closed_states:
        pos_i_up, pos_j_up = state_u.agent_idx
        viz.board.itemconfigure(viz.grid_squares[pos_i_up][pos_j_up], fill="red")

    for node in queue:
        state_u: FarmState = cast(FarmState, node.state)
        pos_i_up, pos_j_up = state_u.agent_idx
        viz.board.itemconfigure(viz.grid_squares[pos_i_up][pos_j_up], fill="grey")

    viz.window.update()
    time.sleep(wait)



def search_optimal(state_start: State, env: Environment, viz) -> Optional[List[int]]:
    """ Return an optimal path

    :param state_start: starting state
    :param env: environment
    :param viz: visualization object

    :return: a list of integers representing the actions that should be taken to reach the goal or None if no solution
    """
    # A* search using a priority queue and a hash set to keep track of expanded nodes 
    # the heuristic is manhatten distance
    
    #Information based off of Informed Search Lecture Slides
    start_node = Node(state_start, 0, None, None)
    open = []
    expanded = {}
    heappush(open,(0, start_node))
    while(open):
        current_node = heappop(open)[1]
        if(env.is_terminal(current_node.state)):
            return get_path(current_node)
        expanded[current_node.__hash__] = current_node
        
        for action in env.get_actions(current_node.state):
            child_state, trans_cost = get_next_state_and_transition_cost(env, current_node.state, action)
            child_node = Node(child_state, current_node.path_cost + trans_cost, action, current_node)
            if(child_node.__hash__ not in expanded or (child_node.path_cost < expanded[child_node.__hash__].path_cost)):
                expanded[child_node.__hash__] = child_node
                total_cost = child_node.path_cost + heuristic(child_node.state)
                heappush(open, (total_cost, child_node))
    return None
        
    
    # ATTEMPT TO IMPLEMENT UNIFORM COST SEARCH
    # frontier = []
    # explored = set()
    # start_node = Node(state_start, 0, None, None)
    # heappush(frontier, (start_node.path_cost, start_node))
    # goal_state = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])
    # while frontier:
    #     node = heappop(frontier)[1]
    #     #print("CURRENT NODE STATE: ", node.state.tiles)
        
    #     if(env.is_terminal(node.state)):
    #         #print("GOAL STATE: ", node.state.tiles)
    #         return get_path(node)
    #     if(node.state not in explored):
    #         explored.add(node.state)
    #         for action in env.get_actions(node.state):
    #             child_state, transition_cost = get_next_state_and_transition_cost(env, node.state, action)
    #             child_node = Node(child_state, node.path_cost + transition_cost, action, node)
    #             if(child_state not in explored or child_node.path_cost < node.path_cost):
    #                 heappush(frontier, (child_node.path_cost, child_node))
    #                 #print("CHILD STATE NOW IN HEAP:", child_state.tiles)
    #                 if(env.is_terminal(node.state)):
    #                     return get_path(node)

    #                 visualize_bfs(viz, list(explored), extract_nodes_from_heap(frontier), 0.1)
    

#Using int instead of float made it go slightly faster without having to do float arithmetic
def heuristic(state: State) -> int:
    #Manhattan Distance 
    # https://stackoverflow.com/a/46507245 
    # https://www.growingwiththeweb.com/2012/06/a-pathfinding-algorithm.html#:~:text=Manhattan%20distance,down%2C%20left%2C%20right).
    
    state_tiles = state.tiles.reshape((3, 3))
    goal_state = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8]).reshape((3, 3))
    total_distance = 0
    for i in range(3):
        for j in range(3):
            #bypassing the 0 tile
            if state_tiles[i, j] != 0:
                goal_position = np.where(goal_state == state_tiles[i, j])
                total_distance += abs(i - goal_position[0][0]) + abs(j - goal_position[1][0])
    return total_distance

#NOT USED FAILD IMPLEMENTATION
def depth_limited_search(state: State, env: Environment, limit: int, path=[]) -> Tuple[Optional[State], List[int]]:
    """
    Perform Depth-Limited Search.
    
    :param node: The current node being explored.
    :param env: The environment in which to search.
    :param limit: The current depth limit for the search.
    :param path: The path (list of actions) taken to reach the current node.
    :return: A tuple with the goal node and list of actions if the goal is found, otherwise None.
    """
    
    if env.is_terminal(state):  # Goal check.
        return state, path
    elif limit == 0:  # Depth limit reached.
        return None, []
    else:
        for someAction in env.get_actions(state):
            child_state, transition_cost = get_next_state_and_transition_cost(env, state, someAction)
            
            if child_state is not None:
                result, actions = depth_limited_search(child_state, env, limit - 1, path + [someAction])
                if result is not None:
                    return result, actions
    return None,[]
        
#NOT USED FAILD IMPLEMENTATION
def get_node_depth(node):
    depth = 0
    while node.parent is not None:
        depth += 1
        node = node.parent
    return depth
#WAS USED TO VISUALIZE BFS BUT NOT USED ANYMORE
def extract_nodes_from_heap(heap):
    return [node for path_cost, node in heap]


def get_path(goal_node: Node) -> List[int]:
    """
    get a path (list of actions) from the start state to the goal state.

    :param goal_node: The goal node from which to get the path.
    :return: List of actions from start to goal.
    """
    actions = []
    while goal_node.parent is not None:
        actions.append(goal_node.parent_action)
        goal_node = goal_node.parent
    actions.reverse()
    return actions


def search_speed(state_start: State, env: Environment, viz) -> Optional[List[int]]:
    """ Return a path as quickly as possible

    :param state_start: starting state
    :param env: environment
    :param viz: visualization object

    :return: a list of integers representing the actions that should be taken to reach the goal or None if no solution
    """
    # Some form of A* but not considering any path cost and just transition cost and heuristic
    # Took out operations that added more computation time, less operations = faster
    start_node = Node(state_start, 0, None, None)
    open = []
    expanded = {}
    heappush(open,(0, start_node))
    
    while(open):
        current_node = heappop(open)[1]
        if(env.is_terminal(current_node.state)):
            return get_path(current_node)
        expanded[current_node.__hash__] = current_node
        
        for action in env.get_actions(current_node.state):
            child_state, trans_cost = get_next_state_and_transition_cost(env, current_node.state, action)
            child_node = Node(child_state, trans_cost, action, current_node)
            if(child_node.__hash__ not in expanded):
                expanded[child_node.__hash__] = child_node
                total_cost = heuristic(child_node.state)
                heappush(open, (total_cost, child_node))
