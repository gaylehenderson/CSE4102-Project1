from __future__ import annotations
from pathlib import Path
from .location import GridLocation
from .mission import MissionMap

def load_mission(path: str | Path) -> MissionMap:
    path = Path(path)
    raw_lines = path.read_text(encoding="utf-8").splitlines()
    battery = 50
    grid_lines: list[str] = []
    for raw in raw_lines:
        line = raw.rstrip("\n")
        if not line.strip():
            continue
        if line.lstrip().startswith("# ") or line.lstrip().startswith("//"):
            continue
        if ":" in line and not any(ch in line for ch in "#.RSABCDEFfrwmc"):
            key, value = line.split(":", 1)
            if key.strip().lower() == "battery":
                battery = int(value.strip())
            continue
        if line.lower().startswith("battery:"):
            battery = int(line.split(":", 1)[1].strip())
            continue
        grid_lines.append(line)

    if not grid_lines:
        raise ValueError(f"Mission file {path} does not contain a grid.")

    start = None
    survivors: dict[str, GridLocation] = {}
    beacons: dict[str, GridLocation] = {}
    survivor_count = 1
    grid: list[list[str]] = []

    for r, line in enumerate(grid_lines):
        row = []
        for c, ch in enumerate(line):
            loc = GridLocation(r, c)
            if ch == "R":
                if start is not None:
                    raise ValueError("Mission maps may contain only one R start tile.")
                start = loc
                row.append("R")
            elif ch == "S":
                label = f"S{survivor_count}"
                survivor_count += 1
                survivors[label] = loc
                row.append("S")
            elif ch in "ABCDEF":
                beacons[ch] = loc
                row.append(ch)
            else:
                row.append(ch)
        grid.append(row)

    if start is None:
        raise ValueError("Mission map must include an R start tile.")
    return MissionMap(grid, start, battery, survivors, beacons, name=path.stem)

def resolve_map_path(name: str | Path) -> Path:
    path = Path(name)
    if path.exists():
        return path
    if path.suffix == "":
        path = path.with_suffix(".mission")
    candidate = Path(__file__).resolve().parent.parent / "maps" / path.name
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"Could not find mission map {name!r}.")


def available_maps() -> list[str]:
    """Return built-in mission map names without the .mission suffix."""
    maps_dir = Path(__file__).resolve().parent.parent / "maps"
    if not maps_dir.exists():
        return []
    return sorted(path.stem for path in maps_dir.glob("*.mission"))
