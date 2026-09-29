from __future__ import annotations
from .mission import MissionMap
from .actions import ACTIONS

def replay_plan(mission: MissionMap, actions: list[str]):
    loc = mission.start
    battery = mission.initial_battery
    total_cost = 0
    frames = [(loc, battery, total_cost)]
    for action in actions:
        if action not in ACTIONS:
            raise ValueError(f"Unknown action {action!r}.")
        dr, dc = ACTIONS[action]
        nxt = loc.moved(dr, dc)
        if mission.is_blocked(nxt):
            raise ValueError(f"Action {action!r} moves into a blocked cell at {nxt}.")
        step_cost = mission.movement_cost(nxt)
        battery -= step_cost
        total_cost += step_cost
        if battery < 0:
            raise ValueError("Plan depleted the robot battery.")
        loc = nxt
        battery = mission.recharge_after_entering(loc, battery)
        frames.append((loc, battery, total_cost))
    return frames
