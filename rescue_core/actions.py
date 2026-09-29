from __future__ import annotations

ACTIONS = {
    "N": (-1, 0),
    "S": (1, 0),
    "W": (0, -1),
    "E": (0, 1),
}

ACTION_NAMES = {
    "N": "north",
    "S": "south",
    "W": "west",
    "E": "east",
}

def all_actions() -> tuple[str, ...]:
    return ("N", "S", "W", "E")
