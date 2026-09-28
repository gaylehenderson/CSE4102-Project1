# Project 1: Rescue Robot Search and Mission Planning

This project uses Python 3.13. The graphical display uses pygame, but the autograder runs without graphics.

Student-editable files:

- `planner.py`
- `objectives.py`
- `heuristics.py`

Do not modify files in `rescue_core/` or `rescue_graphics/`.

## Install

```bash
python3.13 -m pip install pygame
```

## Run the GUI

```bash
python3.13 rescue.py --map tiny_rescue --planner bfs
python3.13 rescue.py --map fire_escape --planner astar --objective reach
python3.13 rescue.py --map beacon_rooms --planner astar --objective inspect
python3.13 rescue.py --map survivor_cluster --planner greedy --objective rescue-all
python3.13 rescue.py --map route_optimization --planner anneal --objective rescue-all
```

## Run without graphics

```bash
python3.13 rescue.py --map tiny_rescue --planner bfs --no-graphics
```

## Run the local autograder

```bash
python3.13 autograder.py
```

The project is graded out of 30 points: 26 points for code behavior and 4 points for self-reflection.

## Self-reflection blocks

Each question includes a plain-comment self-reflection block in one of the student-editable files. Do not use JSON, YAML, dictionaries, or any special syntax. Write normal comment sentences on the comment lines below `# YOUR RESPONSE:`. The checker also accepts text after the colon on the same marker line, but the next-line format is easier to read and debug.

Each reflection is worth 0.5 point. You can receive full reflection credit whether you used GenAI, artificial intelligence, machine learning, a coding assistant, or no such tool. To receive the point, your response must be non-placeholder, at least 25 words total, clearly address tool use or non-use for that question, explain how you checked your answer, and include at least one concrete project detail such as a function name, file name, map name, test id, or command you ran.

Generic filler such as "I did not use GenAI because I understand the concept" will not receive credit unless it includes concrete project details and verification. Natural phrasings such as "I did not rely on any artificial intelligence tool" or "I used Cursor to debug this function" are acceptable when the rest of the reflection is complete.

| Question | Complete the reflection block in | Implementation note |
|---|---|---|
| Q1 | `planner.py` | frontiers and generic `graph_search` |
| Q2 | `planner.py` | also complete `ReachTargetObjective` in `objectives.py` |
| Q3 | `planner.py` | reuses `ReachTargetObjective` |
| Q4 | `planner.py` | reuses `ReachTargetObjective` and `distance_to_target_heuristic` |
| Q5 | `objectives.py` | complete `InspectLocationsObjective` |
| Q6 | `heuristics.py` | first complete `RescueAllSurvivorsObjective` in `objectives.py` |
| Q7 | `objectives.py` | reuses `RescueAllSurvivorsObjective` and adds greedy planning |
| Q8 | `planner.py` | reuses `RescueAllSurvivorsObjective` for route optimization |

Example:

```python
# YOUR RESPONSE:
# I did not use GenAI for Q2 because DFS and BFS were direct wrappers around
# the graph_search function from Q1. I checked my answer by running
# python3.13 autograder.py -q q2 and comparing the BFS result on tiny_rescue
# with the expected path in Q2.1.
```

## Objective methods used by planners

Planners do not read the map directly. They ask an objective for:

- `initial_state()`: the starting `MissionState`
- `is_goal(state)`: whether the current state solves the objective
- `successors(state)`: a list of legal `Transition` objects

In `objectives.py`, use `mission.legal_neighbors(state.robot)` to generate legal moves and `mission.recharge_after_entering(next_location, battery)` after paying the movement cost. Each successor should create a `Transition(action, next_state, cost)`.

Implementation order:

- Before Q2 tests can pass, complete `ReachTargetObjective.initial_state`, `ReachTargetObjective.is_goal`, and `ReachTargetObjective.successors`.
- For Q5, complete `InspectLocationsObjective`.
- Before Q6 tests can pass, complete `RescueAllSurvivorsObjective`; Q6 uses it to test the rescue-all heuristic. Q7 and Q8 reuse it.

## Understanding autograder failures

When a public test fails, the autograder prints the expected behavior and a likely place to look. For example, Q2.1 expects exactly `["E", "E", "E", "E"]` on `tiny_rescue`.

| Failed test | Expected behavior | Start checking |
|---|---|---|
| Q1.1 | Stack pops LIFO and queue pops FIFO | `StackFrontier`, `QueueFrontier` |
| Q1.2 | Priority frontier pops lowest priority, `cheap` before `expensive` | `PriorityFrontier`, `heapq` |
| Q1.3 | `graph_search` returns exactly `["toB", "toD"]` on a toy graph | `graph_search`, path costs, priority |
| Q1.4 | `graph_search` returns exactly `["E", "E"]` and ignores lower-battery repeats | reached/dominated-state logic |
| Q2.1 | BFS returns exactly `["E", "E", "E", "E"]` on `tiny_rescue` | `ReachTargetObjective`, then `breadth_first_plan`, `QueueFrontier` |
| Q2.2 | DFS returns any legal plan that reaches `S1` | `ReachTargetObjective`, then `depth_first_plan`, `StackFrontier` |
| Q3.1 | UCS reaches `S1` on `rubble_corridor` with cost `8` | `ReachTargetObjective`, `uniform_cost_plan`, path-cost priority |
| Q3.2 | UCS visits the charger on `battery_challenge` with cost `30` | `ReachTargetObjective.successors`, battery, charger, UCS |
| Q4.1 | A* reaches `S1` on `fire_escape` | `ReachTargetObjective`, `astar_plan` |
| Q4.2 | A* cost matches UCS cost | `ReachTargetObjective`, A* priority and heuristic |
| Q5.1-Q5.3 | Inspection reaches all beacons and leaves no remaining labels | `InspectLocationsObjective` |
| Q6.1-Q6.4 | Heuristics are consistent and reduce A* node expansions on `large_survivor_cluster` | `RescueAllSurvivorsObjective`, `heuristics.py`, `graph_search` |
| Q7.1-Q7.2 | Greedy planner rescues all survivors without depleting battery | `RescueAllSurvivorsObjective`, `GreedyRescuePlanner` |
| Q8.1-Q8.4 | Simulated annealing finds a better rescue-all route on `route_optimization` | `SimulatedAnnealingRescuePlanner` |
| Q1.R-Q8.R | Plain-comment self-reflection block has 25+ words, AI/coding-assistant use or non-use disclosure, verification, and a concrete project detail | the file listed in the self-reflection table |


## Manual GUI preview

You can explore any mission map manually before implementing the search planners:

```bash
python3.13 rescue.py --map tiny_rescue --manual
python3.13 rescue.py --map beacon_rooms --manual
python3.13 rescue.py --map battery_challenge --manual
```

Controls:

- Arrow keys or WASD move the robot.
- R resets the mission.
- Q or Escape quits.

Manual mode is for visualization only. It does not affect autograder scoring and does not require `planner.py`, `objectives.py`, or `heuristics.py` to be completed.


## GUI legend

The pygame interface uses an original cute rescue-robot design and a compact color-coded key in the bottom status panel.

- Yellow tracked robot: RescueBot.
- Purple radio rings: emergency inspection beacons.
- Person icons with a first-aid marker: survivors.
- Blue arrows: planned route or manual trail.
- Sand dunes: sand, movement cost 2.
- Stacked stone pile: rubble, movement cost 3.
- Blue wave tile: water, movement cost 4.
- Orange flame tile: fire/smoke, movement cost 8.
- Green lightning tile: charging station.

The bottom status panel shows mission cost, battery, score, rescued survivors, and inspected beacons.

Sand and rubble are different terrain types. In the map text files, `m` means sand and costs 2 battery/cost units to enter, while `r` means rubble and costs 3. In the GUI, sand is shown as small yellow sand dunes and rubble is the stacked stone pile. DFS and BFS should not optimize for these terrain costs, but UCS and A* should.


## Battery and charging stations

Charging stations are shown as green tiles and represented by `c` in mission maps. The robot must spend the movement cost to enter the tile; after it reaches the station, its battery is restored to the mission's starting capacity. The `battery_challenge` map is designed to contrast search behavior without long runs of one terrain type: DFS tends to follow the upper east-first branch, BFS finds the shorter middle branch with scattered hazards, and UCS/A* should prefer the longer but lower-cost branch through the green charging station.
