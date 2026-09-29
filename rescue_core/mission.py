from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable
from .location import GridLocation
from .actions import ACTIONS

TERRAIN_COSTS = {
    ".": 1,
    "R": 1,
    "S": 1,
    "A": 1, "B": 1, "C": 1, "D": 1, "E": 1, "F": 1,
    "r": 3,   # rubble
    "w": 4,   # water
    "f": 8,   # fire / extreme danger
    "m": 2,   # sand
    "c": 1,   # charging station
}

TERRAIN_NAMES = {
    ".": "clear floor",
    "R": "start",
    "S": "survivor",
    "A": "inspection beacon", "B": "inspection beacon",
    "C": "inspection beacon", "D": "inspection beacon",
    "E": "inspection beacon", "F": "inspection beacon",
    "r": "rubble",
    "w": "water",
    "f": "fire zone",
    "m": "sand",
    "c": "charging station",
}

@dataclass(frozen=True)
class MissionState:
    """State used by mission objectives.

    remaining is a frozenset of target labels, not a tuple of positions.
    battery is part of the state so terrain and energy constraints matter.
    """
    robot: GridLocation
    remaining: frozenset[str]
    battery: int

@dataclass(frozen=True)
class Transition:
    action: str
    next_state: MissionState
    cost: int

class MissionMap:
    def __init__(self, grid: list[list[str]], start: GridLocation, battery: int,
                 survivors: dict[str, GridLocation], beacons: dict[str, GridLocation],
                 name: str = "mission"):
        self.grid = grid
        self.start = start
        self.initial_battery = battery
        self.survivors = dict(survivors)
        self.beacons = dict(beacons)
        self.name = name
        self.height = len(grid)
        self.width = max(len(row) for row in grid)

    def in_bounds(self, loc: GridLocation) -> bool:
        return 0 <= loc.row < self.height and 0 <= loc.col < len(self.grid[loc.row])

    def tile_at(self, loc: GridLocation) -> str:
        if not self.in_bounds(loc):
            return "#"
        return self.grid[loc.row][loc.col]

    def is_blocked(self, loc: GridLocation) -> bool:
        return self.tile_at(loc) == "#"

    def is_charger(self, loc: GridLocation) -> bool:
        return self.tile_at(loc) == "c"

    def movement_cost(self, loc: GridLocation) -> int:
        return TERRAIN_COSTS.get(self.tile_at(loc), 1)

    def recharge_after_entering(self, loc: GridLocation, battery: int) -> int:
        """Apply charging-station behavior after paying the entry cost.

        A charging station restores the robot to the mission's initial battery
        capacity. The robot must still have enough battery to enter the tile.
        """
        if self.is_charger(loc):
            return self.initial_battery
        return battery

    def target_locations(self, labels: Iterable[str]) -> dict[str, GridLocation]:
        merged = {}
        merged.update(self.survivors)
        merged.update(self.beacons)
        return {label: merged[label] for label in labels}

    def legal_neighbors(self, loc: GridLocation):
        for action, (dr, dc) in ACTIONS.items():
            nxt = loc.moved(dr, dc)
            if self.in_bounds(nxt) and not self.is_blocked(nxt):
                yield action, nxt, self.movement_cost(nxt)
