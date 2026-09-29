
from __future__ import annotations
from dataclasses import dataclass, field
from itertools import count
import heapq
import math
import random
from collections import deque
from typing import Callable, Protocol, Any
from heuristics import distance_to_target_heuristic, remaining_inspection_heuristic

class RescueObjective(Protocol):
    def initial_state(self): ...
    def is_goal(self, state) -> bool: ...
    def successors(self, state): ...

LAST_SEARCH_STATS = {"expanded": 0}

def reset_search_stats():
    """Reset counters from the most recent graph_search call."""
    LAST_SEARCH_STATS["expanded"] = 0

def get_last_search_stats():
    """Return a copy of the most recent graph_search counters."""
    return dict(LAST_SEARCH_STATS)

@dataclass(order=True)
class SearchNode:
    """A node stored in a search frontier.

    A SearchNode is not the same thing as a mission state. The state is the
    world configuration, while the node also remembers how we reached it.

    priority:
        Used only by PriorityFrontier. UCS uses path cost as priority.
        A* uses path cost + heuristic as priority.
    state:
        The current search state. In real missions this is a MissionState.
    plan:
        The list of actions taken from the start state to this node.
    path_cost:
        The sum of movement costs along the plan so far.
    """
    priority: float
    state: Any = field(compare=False)
    plan: list[str] = field(compare=False, default_factory=list)
    path_cost: float = field(compare=False, default=0)

# === Q1 SELF-REFLECTION (0.5 point) ===
# Write 3-5 sentences below, at least 25 words total.
# Say whether you used GenAI, artificial intelligence,
# machine learning, or a coding assistant for this question.
# If yes, name the tool and summarize or copy the prompt(s).
# If no, explain why not.
# Say how you checked your answer.
# Include at least one concrete project detail, such as a function name,
# file name, map name, test id, or command you ran.
#
# Put your answer on the comment lines below the YOUR RESPONSE marker.
# YOUR RESPONSE:
#
# === END Q1 SELF-REFLECTION ===

class StackFrontier:
    """Last-in, first-out frontier used for depth-first planning."""
    def __init__(self):
        # TODO: initialize storage for nodes.
        #
        # Hint: a Python list is a good stack. Store it on self so that
        # push, pop, and empty can all use the same container.
        #
        # Example shape:
        #   self._items = ...
        #
        # In an instance method, self means "this particular StackFrontier
        # object." It is how the object remembers its own data.
        
        self._items = []

    def push(self, node: SearchNode):
        # TODO: add node to the frontier.
        #
        # Hint: for a list-based stack, add new items at the end.
        self._items.append(node)

    def pop(self) -> SearchNode:
        # TODO: remove and return the next node.
        #
        # Hint: DFS needs last-in, first-out behavior. If a list contains
        # [first, second], the next pop should return second.
        return self._items.pop()

    def empty(self) -> bool:
        # TODO: return True when no nodes remain.
        #
        # Hint: empty containers are falsey in Python, so either
        # len(self._items) == 0 or not self._items can work.
        if len(self._items) == 0:
            return True
        else:
            return False

class QueueFrontier:
    """First-in, first-out frontier used for breadth-first planning."""
    def __init__(self):
        # TODO: initialize storage for nodes.
        #
        # Hint: collections.deque is imported above and is designed for
        # efficient queue operations. A deque supports append(...) on the
        # right side and popleft(...) on the left side.
        self._items = deque()

    def push(self, node: SearchNode):
        # TODO: add node to the frontier.
        #
        # Hint: for a deque-based queue, append new nodes on the right.
        
        self._items.append(node)
    def pop(self) -> SearchNode:
        # TODO: remove and return the next node.
        #
        # Hint: BFS needs first-in, first-out behavior. If a queue receives
        # first and then second, the next pop should return first.
        
        return self._items.popleft()
    def empty(self) -> bool:
        # TODO: return True when no nodes remain.
        if len(self._items) == 0:
            return True
        else:
            return False

class PriorityFrontier:
    """Lowest-priority-first frontier used for cost-sensitive planning."""
    def __init__(self):
        # TODO: initialize storage and tie-break counter.
        #
        # Hint: heapq is imported above. It operates on a normal Python list.
        # You will also want count(), imported from itertools, so ties between
        # equal priorities are popped in insertion order.
        #
        # Why a counter? If two nodes have the same priority, Python should not
        # try to compare their states or plans. Store heap entries like:
        #   (priority, tie_break_number, node)
        self.heap = []
        self.counter = count()
        

    def push(self, node: SearchNode):
        # TODO: add node using node.priority.
        #
        # Hint: heapq.heappush(heap_list, item) inserts one item while keeping
        # the smallest item ready to pop. The first tuple element should be
        # node.priority.
        heapq.heappush(self.heap, (node.priority, next(self.counter), node))
        

    def pop(self) -> SearchNode:
        # TODO: remove and return the lowest-priority node.
        #
        # Hint: heapq.heappop(...) returns the whole tuple you pushed. Return
        # only the SearchNode part, not the priority or counter.
        return heapq.heappop(self.heap)[2]
        

    def empty(self) -> bool:
        # TODO: return True when no nodes remain.
        return not self.heap

def state_key(state):
    """Return the mission-progress part of a state.

    MissionState contains battery. A state with the same robot location and the
    same remaining targets but *less* battery is worse than one with more battery.
    Use this key to avoid wasteful cycles such as E,W,E,W that only drain energy.

    For non-mission toy states used by the autograder, returning the state itself
    is fine.
    """
    if hasattr(state, "robot") and hasattr(state, "remaining"):
        return (state.robot, state.remaining)
    return state

def battery_level(state):
    """Return state.battery when present; otherwise return None."""
    return getattr(state, "battery", None)

def is_dominated(records, path_cost, battery):
    """Optional helper for graph_search.

    records is intended to be a list of pairs:
        [(old_path_cost, old_battery_level), ...]

    A new state is dominated when an old record for the same state_key is at
    least as good in every relevant way:

    - old_path_cost <= new path_cost
    - old_battery_level >= new battery, for mission states with battery

    If battery is None, the autograder is probably using a simple toy state
    rather than a MissionState. In that case, cost alone is enough.

    You may use this helper, rename it, or implement the same logic directly
    inside graph_search.
    """
    # TODO: return True if one of the old records dominates the new state.
    raise NotImplementedError

def update_records(records, path_cost, battery):
    """Optional helper for graph_search.

    After deciding that a new state is worth keeping, update the list of best
    records for this state_key.

    Good practice:
    - remove old records that are now dominated by the new one
    - then append the new (path_cost, battery) pair

    This keeps reached from growing with clearly worse versions of the same
    mission-progress state.
    """
    # TODO: return the updated records list.
    raise NotImplementedError

def graph_search(objective: RescueObjective, frontier, priority_fn: Callable[[SearchNode], float] | None = None) -> list[str]:
    """Generic graph search.

    The objective supplies states and transitions. The frontier determines the
    search strategy. The function returns a list of actions such as ["E", "E", "S"].

    Important for this project: do not treat two MissionState objects as equally
    useful just because they are different Python objects. If they have the same
    robot location and same remaining targets, the one with less battery is
    dominated and should usually be ignored.
    """
    # TODO: implement graph search.
    #
    # Suggested structure:
    #
    # 1. Ask the objective for the start state:
    #       start = objective.initial_state()
    
    start = objective.initial_state()
    
    # 2. Wrap the start state in a SearchNode. At the start, the plan is []
    #    and path_cost is 0.
    
    start_node = SearchNode(state = start, plan = [], path_cost = 0,priority=0)
    if priority_fn is not None:
        start_node.priority = priority_fn(start_node)
    
    # 3. If priority_fn is not None, compute the start node's priority before
    #    pushing it. This matters for UCS and A*.
    
    # 4. Push the start node into the frontier.
    
    frontier.push(start_node) 
    
    # 5. Create a reached dictionary. A useful shape is:
    #       reached[state_key(state)] = [(path_cost, battery_level), ...]

    #    Why a list of records? With battery, there may be more than one
    #    non-dominated way to reach the same location and remaining targets.
    
    
    reached = {}
    reached[state_key(start_node.state)] = [(start_node.path_cost, battery_level(start_node.state))]

    while not frontier.empty():
        node = frontier.pop()

        key = state_key(node.state)
        battery = battery_level(node.state)
        records = reached.get(key, [])

        # Skip nodes that have been dominated since insertion
        if (node.path_cost, battery) not in records:
            continue

        # Check whether this is a goal
        if objective.is_goal(node.state):
            return node.plan

        # Expand the node
        LAST_SEARCH_STATS["expanded"] += 1

        for transition in objective.successors(node.state):
            child_state = transition.next_state
            child_plan = node.plan + [transition.action]
            child_cost = node.path_cost + transition.cost

            #child = SearchNode(child_state, child_plan, child_cost)
            
            child = SearchNode(priority=0,state=child_state,plan=child_plan,path_cost=child_cost)

            if priority_fn is not None:
                child.priority = priority_fn(child)

            child_key = state_key(child.state)
            child_battery = battery_level(child.state)
            
           # print("STATE:", child.state)
           # print("ROBOT:", child.state.robot, type(child.state.robot))
           # print("REMAINING:", child.state.remaining, type(child.state.remaining))
           # print("KEY:", child_key)

            # Get existing routes to this state
            existing = reached.get(child_key, [])

            # Check whether an existing route dominates this child
            dominated = any(
                cost <= child_cost and battery >= child_battery
                for cost, battery in existing
            )

            if dominated:
                continue

            # Remove routes dominated by this child
            existing = [
                (cost, battery)
                for cost, battery in existing
                if not (child_cost <= cost and child_battery >= battery)
            ]

            # Record this child as a non-dominated route
            existing.append((child_cost, child_battery))
            reached[child_key] = existing

            # Add the child to the frontier
            frontier.push(child)

    raise ValueError("No solution found")
    # 6. While the frontier is not empty:
    #       node = frontier.pop()
    #
    #    If this node has become dominated by a better route since it was
    #    inserted, skip it.
    #
    # 7. Check for the goal after popping:
    #       if objective.is_goal(node.state): return node.plan
    #
    # 8. Otherwise expand the node:
    #       LAST_SEARCH_STATS["expanded"] += 1
    #       for transition in objective.successors(node.state):
    #
    #    A node is counted as expanded when you call objective.successors(...)
    #    on it. Goal nodes and skipped dominated nodes should not count as
    #    expanded.
    #
    #    Each transition has:
    #       transition.action
    #       transition.next_state
    #       transition.cost
    #
    # 9. Create a child SearchNode:
    #       child plan = node.plan + [transition.action]
    #       child path_cost = node.path_cost + transition.cost
    #
    # 10. If priority_fn is provided, use it to set child.priority. If no
    #     priority_fn is provided, the priority can stay 0 because stacks and
    #     queues ignore it.
    #
    # 11. Use state_key(child.state), battery_level(child.state), and your
    #     dominance logic to decide whether to skip or keep the child.
    #
    # 12. If kept, update reached and push the child into the frontier.
    #
    # If the loop ends without finding a goal, raise ValueError.
    

def depth_first_plan(objective: RescueObjective) -> list[str]:
    # === Q2 SELF-REFLECTION (0.5 point) ===
    # Write 3-5 sentences below, at least 25 words total.
    # Say whether you used GenAI, artificial intelligence,
    # machine learning, or a coding assistant for this question.
    # If yes, name the tool and summarize or copy the prompt(s).
    # If no, explain why not.
    # Say how you checked your answer.
    # Include at least one concrete project detail, such as a function name,
    # file name, map name, test id, or command you ran.
    #
    # Put your answer on the comment lines below the YOUR RESPONSE marker.
    # YOUR RESPONSE:
    #
    # === END Q2 SELF-REFLECTION ===
    #
    # TODO: call graph_search with a StackFrontier
    return graph_search(objective, StackFrontier())

def breadth_first_plan(objective: RescueObjective) -> list[str]:
    # TODO: call graph_search with a QueueFrontier
    
    return graph_search(objective, QueueFrontier())

def uniform_cost_plan(objective: RescueObjective) -> list[str]:
    # === Q3 SELF-REFLECTION (0.5 point) ===
    # Write 3-5 sentences below, at least 25 words total.
    # Say whether you used GenAI, artificial intelligence,
    # machine learning, or a coding assistant for this question.
    # If yes, name the tool and summarize or copy the prompt(s).
    # If no, explain why not.
    # Say how you checked your answer.
    # Include at least one concrete project detail, such as a function name,
    # file name, map name, test id, or command you ran.
    #
    # Put your answer on the comment lines below the YOUR RESPONSE marker.
    # YOUR RESPONSE:
    #
    # === END Q3 SELF-REFLECTION ===
    #
    # TODO: call graph_search with a PriorityFrontier and path-cost priority
    return graph_search(objective, PriorityFrontier(), lambda n: n.path_cost)

def astar_plan(objective: RescueObjective, heuristic: Callable[[Any, RescueObjective], float]) -> list[str]:
    # === Q4 SELF-REFLECTION (0.5 point) ===
    # Write 3-5 sentences below, at least 25 words total.
    # Say whether you used GenAI, artificial intelligence,
    # machine learning, or a coding assistant for this question.
    # If yes, name the tool and summarize or copy the prompt(s).
    # If no, explain why not.
    # Say how you checked your answer.
    # Include at least one concrete project detail, such as a function name,
    # file name, map name, test id, or command you ran.
    #
    # Put your answer on the comment lines below the YOUR RESPONSE marker.
    # YOUR RESPONSE:
    #
    # === END Q4 SELF-REFLECTION ===
    #
    # TODO: call graph_search with path_cost + heuristic
    return graph_search(objective,PriorityFrontier(),priority_fn=lambda node: node.path_cost + remaining_inspection_heuristic(node.state, objective))

class GreedyRescuePlanner:
    """Repeatedly plan to one target chosen by a heuristic scoring rule."""

    def __init__(self, heuristic):
        self.heuristic = heuristic

    def plan(self, objective) -> list[str]:
        # TODO: repeatedly choose and complete a remaining target until done.
        # This intentionally differs from older grid projects: objectives expose
        # labeled rescue targets and battery-aware MissionState objects.
        raise NotImplementedError

class SimulatedAnnealingRescuePlanner:
    """Use local search to improve the order of multi-survivor rescue targets."""

    def __init__(self, seed: int = 0, iterations: int = 1000,
                 initial_temperature: float = 20.0, cooling_rate: float = 0.995):
        self.seed = seed
        self.iterations = iterations
        self.initial_temperature = initial_temperature
        self.cooling_rate = cooling_rate

    def plan(self, objective) -> list[str]:
        # === Q8 SELF-REFLECTION (0.5 point) ===
        # Write 3-5 sentences below, at least 25 words total.
        # Say whether you used GenAI, artificial intelligence,
        # machine learning, or a coding assistant for this question.
        # If yes, name the tool and summarize or copy the prompt(s).
        # If no, explain why not.
        # Say how you checked your answer.
        # Include at least one concrete project detail, such as a function name,
        # file name, map name, test id, or command you ran.
        #
        # Put your answer on the comment lines below the YOUR RESPONSE marker.
        # YOUR RESPONSE:
        #
        # === END Q8 SELF-REFLECTION ===
        #
        # TODO: optimize the order in which remaining survivors are rescued.
        #
        # Suggested approach:
        #
        # 1. Represent a local-search state as a tuple of survivor labels, e.g.
        #       ("S2", "S4", "S1", "S3")
        #
        # 2. Write an evaluation helper that converts an order into an actual
        #    action plan. For each label in the order, plan one leg from the
        #    current MissionState to that survivor. The _SingleLegObjective
        #    below is designed for that. Use uniform_cost_plan for each leg.
        #
        # 3. If a leg is impossible, treat that order as having infinite cost.
        #
        # 4. Generate neighboring orders by swapping two labels, reversing a
        #    segment, or moving one label to another position.
        #
        # 5. Use simulated annealing: always accept a better order, and sometimes
        #    accept a worse order with probability based on temperature:
        #       math.exp(-(new_cost - old_cost) / temperature)
        #
        # 6. Use random.Random(self.seed) instead of the global random module so
        #    your planner is deterministic for the autograder.
        #
        # 7. Return the action list for the best order found.
        raise NotImplementedError

class _SingleLegObjective:
    """Adapter for planning from a current state to one target label."""

    def __init__(self, parent, start_state, label):
        self.parent = parent
        self._start_state = start_state
        self.label = label
        self.target_location = parent.locations[label]

    def initial_state(self):
        return self._start_state

    def is_goal(self, state):
        return state.robot == self.target_location

    def successors(self, state):
        return self.parent.successors(state)
