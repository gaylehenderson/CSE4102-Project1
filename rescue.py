from __future__ import annotations
import argparse
from rescue_core.parser import load_mission, resolve_map_path, available_maps
from rescue_core.simulator import replay_plan
from objectives import ReachTargetObjective, InspectLocationsObjective, RescueAllSurvivorsObjective
from planner import depth_first_plan, breadth_first_plan, uniform_cost_plan, astar_plan, GreedyRescuePlanner, SimulatedAnnealingRescuePlanner
from heuristics import zero_heuristic, distance_to_target_heuristic, remaining_inspection_heuristic, remaining_survivor_heuristic, rescue_priority_score

PLANNERS = {
    "dfs": depth_first_plan,
    "bfs": breadth_first_plan,
    "ucs": uniform_cost_plan,
}

def choose_objective(mission, name: str, target: str | None):
    if name == "reach":
        label = target or (sorted(mission.survivors)[0] if mission.survivors else sorted(mission.beacons)[0])
        return ReachTargetObjective(mission, label)
    if name == "inspect":
        return InspectLocationsObjective(mission)
    if name == "rescue-all":
        return RescueAllSurvivorsObjective(mission)
    raise ValueError(f"Unknown objective {name!r}")

def main():
    parser = argparse.ArgumentParser(description="4102 Rescue Robot Search")
    parser.add_argument("--map", "--layout", dest="map_name", default="tiny_rescue",
                        help="Mission map name or path, e.g., tiny_rescue or maps/tiny_rescue.mission")
    parser.add_argument("--objective", choices=["reach", "inspect", "rescue-all"], default="reach")
    parser.add_argument("--target", default=None, help="Target label such as S1 or A")
    parser.add_argument("--planner", "--algorithm", choices=["dfs", "bfs", "ucs", "astar", "greedy", "anneal"], default="bfs")
    parser.add_argument("--no-graphics", action="store_true")
    parser.add_argument("--manual", action="store_true",
                        help="Open an interactive keyboard-controlled Rescue Robot demo. "
                             "This mode does not require planner.py to be implemented.")
    parser.add_argument("--frame-time", type=float, default=0.08)
    parser.add_argument("--list-maps", action="store_true", help="List available mission maps and exit.")
    args = parser.parse_args()

    if args.list_maps:
        print("Available mission maps:")
        for name in available_maps():
            print(f"  {name}")
        return

    mission = load_mission(resolve_map_path(args.map_name))

    if args.manual:
        if args.no_graphics:
            raise ValueError("--manual requires graphics; remove --no-graphics.")
        from rescue_graphics.display import manual_play
        manual_play(mission)
        return

    objective = choose_objective(mission, args.objective, args.target)

    if args.planner == "astar":
        if args.objective == "reach":
            plan = astar_plan(objective, distance_to_target_heuristic)
        elif args.objective == "inspect":
            plan = astar_plan(objective, remaining_inspection_heuristic)
        else:
            plan = astar_plan(objective, remaining_survivor_heuristic)
    elif args.planner == "greedy":
        plan = GreedyRescuePlanner(rescue_priority_score).plan(objective)
    elif args.planner == "anneal":
        plan = SimulatedAnnealingRescuePlanner(seed=0, iterations=1000).plan(objective)
    else:
        plan = PLANNERS[args.planner](objective)

    frames = replay_plan(mission, plan)

    # Summarize mission progress so non-graphics runs clearly report
    # whether multi-target objectives were completed.
    visited_locations = [frame[0] for frame in frames]
    rescued = {
        label for label, loc in mission.survivors.items()
        if loc in visited_locations
    }
    inspected = {
        label for label, loc in mission.beacons.items()
        if loc in visited_locations
    }

    print(f"Plan length: {len(plan)}")
    print(f"Plan actions: {' '.join(plan)}")
    print(f"Mission cost: {frames[-1][2]}")
    print(f"Battery remaining: {frames[-1][1]}")
    print(f"Survivors rescued: {len(rescued)}/{len(mission.survivors)}")
    print(f"Beacons inspected: {len(inspected)}/{len(mission.beacons)}")

    if not args.no_graphics:
        from rescue_graphics.display import animate
        animate(mission, frames, delay=args.frame_time)

if __name__ == "__main__":
    main()
