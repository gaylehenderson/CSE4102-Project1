
from __future__ import annotations
from rescue_core.mission import MissionState

def zero_heuristic(state: MissionState, objective) -> float:
    return 0

def distance_to_target_heuristic(state: MissionState, objective) -> float:
    """Admissible and consistent estimate for reaching a single target."""
    # TODO: return Manhattan distance to objective.target_location
    x1 = state.robot.row
    y1 = state.robot.col

    x2 = objective.target_location.row
    y2 = objective.target_location.col

    return abs(x1 - x2) + abs(y1 - y2)

# === Q6 SELF-REFLECTION (0.5 point) ===
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
# === END Q6 SELF-REFLECTION ===

def remaining_inspection_heuristic(state: MissionState, objective) -> float:
    """Consistent estimate for visiting all remaining inspection locations."""
    # TODO: estimate remaining cost without overestimating.
    #
    # Q6 now checks consistency, which means every legal transition must satisfy:
    #   h(state) <= transition.cost + h(transition.next_state)
    #
    # Your heuristic should also:
    #   - return 0 when state.remaining is empty,
    #   - never return a negative number,
    #   - return a positive value at the start of beacon_rooms.
    #
    # Think about what must still be true before all remaining beacons can be
    # visited. A useful heuristic is a lower bound that uses the geometry of
    # the unvisited locations, not the full cost of an actual route.

    if len(state.remaining) == 0:
        return 0

    max_distance = 0

    for label in state.remaining:

        x1 = state.robot.row
        y1 = state.robot.col
        
        location = objective.mission.beacons[label]

        x2 = location.row
        y2 = location.col

        distance = abs(x1 - x2) + abs(y1 - y2)

        if distance > max_distance:
            max_distance = distance

    return max_distance

def remaining_survivor_heuristic(state: MissionState, objective) -> float:
    """Consistent estimate for rescuing all remaining survivors."""
    # TODO: estimate remaining rescue effort without overestimating.
    #
    # This heuristic is tested with RescueAllSurvivorsObjective from
    # objectives.py. Complete that objective's initial_state, is_goal, and
    # successors methods before debugging this heuristic.
    #
    # Q6 checks the same consistency inequality:
    #   
    #
    # This is the project's most open-ended heuristic challenge. You want a
    # lower bound for the rescue-all routing problem under the battery budget.
    # Stronger consistent lower bounds should make A* expand fewer nodes on
    # large_survivor_cluster.
    
    # here's the inequality; h(state) <= transition.cost + h(transition.next_state)

    if len(state.remaining) == 0:
        return 0

    locations = []

    for label in state.remaining:
        locations.append(objective.mission.beacons[label])

    # Distance from robot to the closest remaining survivor
    closest_distance = float("inf")

    for location in locations:
        distance = (
            abs(state.robot.row - location.row)
            + abs(state.robot.col - location.col)
        )

        if distance < closest_distance:
            closest_distance = distance

    # Build a minimum spanning tree among remaining survivors
    connected = [locations[0]]
    unconnected = locations[1:]

    mst_distance = 0

    while len(unconnected) > 0:
        smallest_distance = float("inf")
        closest_location = None

        for connected_location in connected:
            for location in unconnected:
                distance = (abs(connected_location.row - location.row) + abs(connected_location.col - location.col))

                if distance < smallest_distance:
                    smallest_distance = distance
                    closest_location = location

        mst_distance += smallest_distance
        connected.append(closest_location)
        unconnected.remove(closest_location)

    return closest_distance + mst_distance

def rescue_priority_score(state: MissionState, label: str, objective) -> float:
    """Lower scores are chosen first by GreedyRescuePlanner."""
    # TODO: score a remaining survivor/inspection target using distance and terrain awareness.
    raise NotImplementedError
