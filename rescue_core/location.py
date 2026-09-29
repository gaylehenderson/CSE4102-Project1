from __future__ import annotations
from dataclasses import dataclass

@dataclass(frozen=True, order=True)
class GridLocation:
    """A row/column location in a rescue mission grid."""
    row: int
    col: int

    def moved(self, drow: int, dcol: int) -> "GridLocation":
        return GridLocation(self.row + drow, self.col + dcol)

    def manhattan_to(self, other: "GridLocation") -> int:
        return abs(self.row - other.row) + abs(self.col - other.col)
