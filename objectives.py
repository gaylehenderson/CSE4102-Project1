
from __future__ import annotations
from rescue_core.mission import MissionMap, MissionState, Transition

class ReachTargetObjective:
    """Reach one labeled target in a mission map.

    Q2 uses this objective before BFS or DFS can do anything useful. If a Q2
    test fails with a MissionState/Transition/successors error, start here
    before debugging your frontier code in planner.py.
    """

    def __init__(self, mission: MissionMap, target_label: str):
        self.mission = mission
        self.target_label = target_label
        locations = mission.target_locations([target_label])
        self.target_location = locations[target_label]

    def initial_state(self) -> MissionState:
        # TODO for Q2:
        # Return a MissionState with three fields:
        #   1. robot: the starting grid location, self.mission.start
        #   2. remaining: a frozenset containing self.target_label
        #   3. battery: the starting battery, self.mission.initial_battery
        
        return MissionState(robot=self.mission.start,remaining=frozenset([self.target_label]),battery=self.mission.initial_battery)
        
    def is_goal(self, state: MissionState) -> bool:
        # TODO for Q2:
        # The single-target mission is done when the robot's location equals
        # self.target_location.
        
        if (state.robot == self.target_location):
            return True
        return False

    def successors(self, state: MissionState) -> list[Transition]:
        # TODO for Q2:
        # Return one Transition for each legal move the robot can afford.
        #
        # Useful API calls:
        #   self.mission.legal_neighbors(state.robot)
        #       yields (action, next_location, movement_cost)
        #   self.mission.recharge_after_entering(next_location, battery)
        #       refills the battery if next_location is a charging station
        #
        # Suggested structure:
        #   - start with an empty list
        #   - for each legal neighbor, subtract movement_cost from state.battery
        #   - skip the move if the battery would become negative
        #   - apply recharge_after_entering after paying the entry cost
        #   - keep remaining as a frozenset; if next_location is the target,
        #     remove self.target_label from it
        #   - build next_state = MissionState(next_location, remaining, new_battery)
        #   - append Transition(action, next_state, movement_cost)
        
        successor_list = []

        for action, next_location, movement_cost in self.mission.legal_neighbors(state.robot):
            new_battery = state.battery - movement_cost

            if new_battery < 0:
                continue

            new_battery = self.mission.recharge_after_entering(
                next_location,
                new_battery
            )

            remaining = state.remaining

            if next_location == self.target_location:
                remaining = frozenset(
                    label for label in remaining
                    if label != self.target_label
                )

            next_state = MissionState(
                robot=next_location,
                remaining=remaining,
                battery=new_battery
            )

            successor_list.append(
                Transition(action, next_state, movement_cost)
            )

        return successor_list
        

# === Q5 SELF-REFLECTION (0.5 point) ===
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
# YOUR RESPONSE: I did use GenAI for this component of the project. I used ChatGPT to figure out the distinction between beacon and locations in the mission.py attributes to I could figure out what was the most appropriate attribute to use here. I used it because it allowed me to gain a deeper understanding of the computational underworkings of the project.
#
# === END Q5 SELF-REFLECTION ===

class InspectLocationsObjective:
    """Visit a set of labeled inspection beacons, in any order."""

    def __init__(self, mission: MissionMap, labels: list[str] | tuple[str, ...] | None = None):
        self.mission = mission
        self.labels = tuple(labels) if labels is not None else tuple(sorted(mission.beacons))
        self.locations = mission.target_locations(self.labels)

    def initial_state(self) -> MissionState:
        # TODO for Q5:
        # Start at self.mission.start with self.mission.initial_battery.
        # The remaining field should be a frozenset of labels that still need
        # to be inspected. If the robot starts on one of those locations, that
        # label is already complete and should not be included.
        
        start = self.mission.start
        battery = self.mission.initial_battery

        remaining = set(self.mission.beacons)

        for label in self.mission.beacons:
            if self.mission.beacons[label] == start:
                remaining.remove(label)

        remaining = frozenset(remaining)

        return MissionState(
            robot=start,
            remaining=remaining,
            battery=battery
        )

    def is_goal(self, state: MissionState) -> bool:
        # TODO for Q5:
        # The objective is complete when there are no labels left in
        # state.remaining.
        return len(state.remaining) == 0

    def successors(self, state: MissionState) -> list[Transition]:
        # TODO for Q5:
        # This is similar to ReachTargetObjective.successors, but after moving
        # you must also update state.remaining. If next_location matches the
        # location of a remaining beacon label, remove that label in the next
        # MissionState. Keep using legal_neighbors, recharge_after_entering,
        # MissionState, and Transition.
        successor_list = []

        for action, next_location, movement_cost in self.mission.legal_neighbors(state.robot):
            new_battery = state.battery - movement_cost

            if new_battery < 0:
                continue

            new_battery = self.mission.recharge_after_entering(
                next_location,
                new_battery
            )

            remaining = set(state.remaining)

            for label in state.remaining:
                if self.mission.beacons[label] == next_location:
                    remaining.remove(label)

            remaining = frozenset(remaining)

            next_state = MissionState(
                robot=next_location,
                remaining=remaining,
                battery=new_battery
            )

            successor_list.append(
                Transition(action, next_state, movement_cost)
            )

        return successor_list

class RescueAllSurvivorsObjective:
    """Reach all survivors, in any order, before the battery runs out.

    Important: implement this before running Q6. The Q6 rescue-all heuristic
    tests need this objective so they can generate legal rescue-all states and
    transitions. Q7 and Q8 reuse the same objective for multi-survivor planners.
    """

    def __init__(self, mission: MissionMap):
        self.mission = mission
        self.labels = tuple(sorted(mission.survivors))
        self.locations = mission.target_locations(self.labels)

    def initial_state(self) -> MissionState:
        # TODO before Q6:
        # Start at self.mission.start with self.mission.initial_battery.
        # The remaining field should be a frozenset of survivor labels that
        # still need rescue. If the robot starts on a survivor location, that
        # survivor is already rescued and should not be included.
        remaining = set()

        for label in self.labels:
            location = self.locations[label]

            if location != self.mission.start:
                remaining.add(label)

        return MissionState(
            robot=self.mission.start,
            battery=self.mission.initial_battery,
            remaining=frozenset(remaining)
        )
        
    def is_goal(self, state: MissionState) -> bool:
        # TODO before Q6:
        # The rescue-all mission is complete when state.remaining is empty.
        return len(state.remaining) == 0

    def successors(self, state: MissionState) -> list[Transition]:
        # TODO before Q6:
        # This has the same shape as InspectLocationsObjective.successors.
        # Generate legal battery-feasible moves with legal_neighbors and
        # recharge_after_entering. Then remove any remaining survivor label
        # whose location equals next_location before creating the next
        # MissionState and Transition.
        result = []

        for action, next_location, movement_cost in self.mission.legal_neighbors(state.robot):
            next_battery = state.battery - movement_cost

            if next_battery < 0:
                continue

            next_battery = self.mission.recharge_after_entering(
                next_location,
                next_battery
            )

            remaining = set(state.remaining)

            for label in state.remaining:
                if self.locations[label] == next_location:
                    remaining.remove(label)

            next_state = MissionState(
                robot=next_location,
                battery=next_battery,
                remaining=frozenset(remaining)
            )

            result.append(
                Transition(
                    action,
                    next_state,
                    movement_cost
                )
            )

        return result

# === Q7 SELF-REFLECTION (0.5 point) ===
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
# YOUR RESPONSE: I used GenAI for this component of the project. I used Claude to help me review the frozenset in successor() because I wasn't too familiar with the concept prior to this project. Again, I wasn't familiar with the concept so I wanted to learn it so I could use it
#
# === END Q7 SELF-REFLECTION ===
