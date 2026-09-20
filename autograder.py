from __future__ import annotations
import argparse, json, traceback
from collections import deque
from pathlib import Path
import re

from rescue_core.parser import load_mission, resolve_map_path
from rescue_core.simulator import replay_plan

TOTAL_POINTS = 30
REFLECTION_POINTS = 0.5
REFLECTION_MIN_WORDS = 25

REFLECTION_LOCATIONS = {
    "q1": "planner.py",
    "q2": "planner.py",
    "q3": "planner.py",
    "q4": "planner.py",
    "q5": "objectives.py",
    "q6": "heuristics.py",
    "q7": "objectives.py",
    "q8": "planner.py",
}

CONCRETE_REFLECTION_TERMS = {
    "planner.py", "objectives.py", "heuristics.py", "autograder.py", "rescue.py",
    "graph_search", "StackFrontier", "QueueFrontier", "PriorityFrontier",
    "depth_first_plan", "breadth_first_plan", "uniform_cost_plan", "astar_plan",
    "GreedyRescuePlanner", "SimulatedAnnealingRescuePlanner", "_SingleLegObjective",
    "ReachTargetObjective", "InspectLocationsObjective", "RescueAllSurvivorsObjective",
    "distance_to_target_heuristic", "remaining_inspection_heuristic",
    "remaining_survivor_heuristic", "rescue_priority_score",
    "MissionState", "heapq", "deque", "BFS", "DFS", "UCS", "A*",
    "tiny_rescue", "small_rescue", "medium_rescue", "large_rescue",
    "rubble_corridor", "battery_challenge", "fire_escape", "beacon_rooms",
    "large_inspection", "survivor_cluster", "large_survivor_cluster",
    "route_optimization",
}

GENAI_DISCLOSURE_PATTERNS = [
    r"\bgen[- ]?ai\b",
    r"\bgenerative ai\b",
    r"\bartificial intelligence\b",
    r"\bmachine learning\b",
    r"\bml\b",
    r"\blarge language models?\b",
    r"\bchatgpt\b",
    r"\bopenai\b",
    r"\bcopilot\b",
    r"\bclaude\b",
    r"\bgemini\b",
    r"\bcursor\b",
    r"\bperplexity\b",
    r"\bdeepseek\b",
    r"\bgrok\b",
    r"\bbard\b",
    r"\bwindsurf\b",
    r"\bmistral\b",
    r"\bllama\b",
    r"\bcodewhisperer\b",
    r"\bamazon q\b",
    r"\btabnine\b",
    r"\bcodeium\b",
    r"\bqodo\b",
    r"\bmeta ai\b",
    r"\bcoding assistants?\b",
    r"\bcode assistants?\b",
    r"\bprogramming assistants?\b",
    r"\bllm\b",
    r"\bai\b",
    r"\bdid not\s+(?:use|rely on|ask|consult)\s+(?:any\s+)?(?:tools?|outside help|external help|coding help|coding assistance|coding assistants?|code assistants?|programming assistants?)\b",
    r"\bdidn't\s+(?:use|rely on|ask|consult)\s+(?:any\s+)?(?:tools?|outside help|external help|coding help|coding assistance|coding assistants?|code assistants?|programming assistants?)\b",
    r"\bwithout\s+(?:using\s+)?(?:any\s+)?(?:tools?|outside help|external help|coding help|coding assistance|coding assistants?|code assistants?|programming assistants?)\b",
    r"\bno\s+(?:tools?|outside help|external help|coding help|coding assistance|coding assistants?|code assistants?|programming assistants?)\b",
]

CASE_HINTS = {
    "Q1.1 Stack and Queue frontiers behave correctly": (
        "Expected stack pop order to be LIFO and queue pop order to be FIFO.",
        "Check StackFrontier and QueueFrontier in planner.py.",
    ),
    "Q1.2 PriorityFrontier uses lowest priority first": (
        "After pushing priority 5 ('expensive') and priority 1 ('cheap'), pop() should return 'cheap'.",
        "Check PriorityFrontier, heapq usage, and the tie-break counter in planner.py.",
    ),
    "Q1.3 graph_search works with cost priority": (
        "Expected action list exactly ['toB', 'toD'] on the toy graph because total cost 4 beats the cost-11 route.",
        "Check graph_search path_cost updates, priority_fn usage, and when nodes are pushed into the frontier.",
    ),
    "Q1.4 graph_search handles dominated battery states": (
        "Expected action list exactly ['E', 'E']; do not revisit the same mission-progress state with lower battery.",
        "Check state_key, battery_level, is_dominated/update_records, and reached handling in graph_search.",
    ),
    "Q2.1 BFS finds shortest plan on tiny_rescue": (
        "Expected result exactly ['E', 'E', 'E', 'E'] on tiny_rescue.",
        "Check ReachTargetObjective.initial_state, is_goal, and successors in objectives.py first; then check breadth_first_plan and QueueFrontier in planner.py.",
    ),
    "Q2.2 DFS returns a valid survivor-reaching plan": (
        "Expected any legal action list that reaches survivor S1 on tiny_rescue.",
        "Check ReachTargetObjective.initial_state, is_goal, and successors in objectives.py first; then check depth_first_plan and StackFrontier in planner.py.",
    ),
    "Q3.1 UCS chooses lower-cost route on rubble_corridor": (
        "Expected a valid plan to S1 with total cost exactly 8 on rubble_corridor.",
        "Check ReachTargetObjective in objectives.py, then check uniform_cost_plan uses PriorityFrontier with node.path_cost as priority.",
    ),
    "Q3.2 UCS uses charger route on battery_challenge": (
        "Expected a valid plan that visits the charging station and has total cost exactly 30.",
        "Check ReachTargetObjective.successors for battery/recharge updates, then check UCS path-cost priority.",
    ),
    "Q4.1 A* reaches the survivor on fire_escape": (
        "Expected a legal A* plan that reaches S1 on fire_escape.",
        "Check ReachTargetObjective in objectives.py, then check astar_plan uses priority path_cost + heuristic(state, objective).",
    ),
    "Q4.2 A* matches UCS optimal cost": (
        "Expected A* total cost to equal UCS total cost on fire_escape.",
        "Check A* priority calculation and that the distance-to-target heuristic does not overestimate.",
    ),
    "Q5.1 Inspection objective reaches all beacons": (
        "Expected the inspection objective to finish with all beacons completed on beacon_rooms.",
        "Check InspectLocationsObjective initial_state, is_goal, and successors in objectives.py.",
    ),
    "Q5.2 Inspection state tracks remaining labels": (
        "Expected final.remaining to be an empty frozenset after all beacons are inspected.",
        "Check that completed beacon labels are removed from the remaining set.",
    ),
    "Q5.3 Inspection route cost is reasonable": (
        "Expected the uniform-cost inspection plan on beacon_rooms to have cost <= 34.",
        "Check transition costs and UCS behavior for the inspection objective.",
    ),
    "Q6.1 Inspection heuristic is consistent": (
        "Expected remaining_inspection_heuristic to be positive at the start, zero at goals, nonnegative, and consistent across legal moves.",
        "Check InspectLocationsObjective in objectives.py first; then check remaining_inspection_heuristic in heuristics.py.",
    ),
    "Q6.2 Rescue-all heuristic is consistent": (
        "Expected remaining_survivor_heuristic to be positive at the start, zero at goals, nonnegative, consistent across legal moves, and not overestimate the true optimal rescue-all cost on survivor_cluster.",
        "Check RescueAllSurvivorsObjective.initial_state, is_goal, and successors in objectives.py first; then check remaining_survivor_heuristic in heuristics.py.",
    ),
    "Q6.3 Rescue-all heuristic beats UCS expansions": (
        "Expected remaining_survivor_heuristic to pass the rescue-all sanity/consistency checks and make A* expand at most 65% as many nodes as the zero heuristic on large_survivor_cluster.",
        "Check RescueAllSurvivorsObjective in objectives.py first; then make remaining_survivor_heuristic more informative while preserving consistency.",
    ),
    "Q6.4 Rescue-all heuristic earns strong expansion score": (
        "Expected remaining_survivor_heuristic to pass the rescue-all sanity/consistency checks and make A* expand at most 35% as many nodes as the zero heuristic on large_survivor_cluster.",
        "After RescueAllSurvivorsObjective works, think about a routing lower bound that accounts for relationships among all remaining survivors.",
    ),
    "Q7.1 Greedy planner rescues all survivors": (
        "Expected GreedyRescuePlanner to return a legal plan that rescues every survivor in survivor_cluster.",
        "Check RescueAllSurvivorsObjective in objectives.py and GreedyRescuePlanner.plan in planner.py.",
    ),
    "Q7.2 Greedy planner returns a feasible budgeted plan": (
        "Expected the greedy rescue plan to finish with battery >= 0 and cost within the initial battery.",
        "Check that RescueAllSurvivorsObjective.successors updates battery/remaining labels, and that greedy planning updates the current state after each leg.",
    ),
    "Q8.1 Annealing planner rescues all survivors": (
        "Expected SimulatedAnnealingRescuePlanner to return a legal plan that rescues all survivors on route_optimization.",
        "Check that your annealing planner converts the best survivor order into an action list.",
    ),
    "Q8.2 Annealing planner is deterministic with a seed": (
        "Expected two runs with the same seed and parameters to return the same action list.",
        "Use random.Random(self.seed), not the global random module.",
    ),
    "Q8.3 Annealing improves over greedy baseline": (
        "Expected simulated annealing to achieve cost <= 48 on route_optimization, beating the greedy baseline cost 52.",
        "Check your neighbor generation, acceptance probability, and best-order tracking.",
    ),
    "Q8.4 Annealing reaches strong optimization target": (
        "Expected simulated annealing to achieve cost <= 40 on route_optimization.",
        "Use enough iterations and a cooling schedule that can escape the initial greedy ordering.",
    ),
}

def result(name, score, max_score, output=""):
    status = "passed" if score == max_score else "failed"
    return {"name": name, "score": score, "max_score": max_score, "status": status, "output": output}

def format_points(value):
    value = float(value)
    if value.is_integer():
        return str(int(value))
    return f"{value:.2f}".rstrip("0").rstrip(".")

class ReflectionError(AssertionError):
    """Raised when a required plain-comment self-reflection is missing."""

def _clean_comment_line(line):
    text = line.strip()
    if text.startswith("#"):
        text = text[1:].strip()
    return text

def _reflection_message(qid, filename, issue, found=None):
    qlabel = qid.upper()
    found_text = f"\n\nFound:\n{found}" if found else ""
    return (
        f"Expected behavior:\n"
        f"Complete the plain-comment self-reflection block for {qlabel}.\n\n"
        f"Where to complete it:\n"
        f"{filename}, inside the block labeled:\n"
        f"=== {qlabel} SELF-REFLECTION (0.5 point) ===\n\n"
        f"What to write:\n"
        f"Write 3-5 sentences, at least {REFLECTION_MIN_WORDS} words total, explaining whether you used GenAI, artificial intelligence, machine learning, or a coding assistant for {qlabel}.\n"
        f"If yes, name the tool, such as ChatGPT, Copilot, Cursor, Perplexity, Claude, Gemini, DeepSeek, Grok, Windsurf, or another tool, and summarize or copy the prompt(s).\n"
        f"If no, explain that you did not use an AI/coding-assistant tool and say why not.\n"
        f"Also say how you checked your answer.\n"
        f"Include at least one concrete project detail, such as a function name, file name, map name, test id, or command.\n\n"
        f"Recommended placement:\n"
        f"Write your answer on the comment lines below # YOUR RESPONSE:. The checker also accepts text after the colon on the same line, but the next-line format is easiest to read.\n\n"
        f"Issue found:\n"
        f"{issue}{found_text}\n\n"
        f"Concrete detail examples:\n"
        f"graph_search, planner.py, Q2.1, tiny_rescue, autograder.py -q {qid}\n\n"
        f"Example:\n"
        f"# YOUR RESPONSE:\n"
        f"# I used ChatGPT to ask how uniform-cost search should prioritize paths.\n"
        f"# My prompt was: \"Explain UCS with a Python priority queue.\"\n"
        f"# I checked my answer by running python3.13 autograder.py -q {qid} and comparing the result on battery_challenge."
    )

def has_genai_disclosure(response):
    lowered = response.lower()
    return any(re.search(pattern, lowered) for pattern in GENAI_DISCLOSURE_PATTERNS)

def has_concrete_project_detail(response):
    lowered = response.lower()
    if any(term.lower() in lowered for term in CONCRETE_REFLECTION_TERMS):
        return True
    if re.search(r"\bq[1-8](?:\.(?:[1-9]|r))?\b", lowered):
        return True
    if re.search(r"\b(?:python3(?:\.13)?|py -3\.13)\b", lowered) and (
        "autograder.py" in lowered or "rescue.py" in lowered
    ):
        return True
    return False

def check_reflection(qid, filename):
    qlabel = qid.upper()
    path = Path(filename)
    if not path.exists():
        raise ReflectionError(_reflection_message(qid, filename, f"{filename} was not found."))

    lines = path.read_text(encoding="utf-8").splitlines()
    start_text = f"=== {qlabel} SELF-REFLECTION"
    end_text = f"=== END {qlabel} SELF-REFLECTION ==="

    start_idx = next((i for i, line in enumerate(lines) if start_text in line), None)
    if start_idx is None:
        raise ReflectionError(_reflection_message(
            qid, filename, f"The {qlabel} self-reflection block is missing."
        ))

    end_idx = next((i for i in range(start_idx + 1, len(lines)) if end_text in lines[i]), None)
    if end_idx is None:
        raise ReflectionError(_reflection_message(
            qid, filename, f"The {qlabel} self-reflection block has no END marker."
        ))

    response_idx = next((i for i in range(start_idx + 1, end_idx) if "YOUR RESPONSE:" in lines[i]), None)
    if response_idx is None:
        raise ReflectionError(_reflection_message(
            qid, filename, "The block is present, but the YOUR RESPONSE marker is missing."
        ))

    marker_text = _clean_comment_line(lines[response_idx])
    same_line_response = marker_text.split("YOUR RESPONSE:", 1)[1].strip()

    response_lines = []
    if same_line_response:
        response_lines.append(same_line_response)
    response_lines.extend(_clean_comment_line(line) for line in lines[response_idx + 1:end_idx])
    response_lines = [
        line for line in response_lines
        if line and not line.startswith("===") and "YOUR RESPONSE:" not in line
    ]
    response = " ".join(response_lines)
    response = re.sub(r"\s+", " ", response).strip()

    placeholders = {
        "todo", "tbd", "n/a", "na", "none", "no", "...", ".", "placeholder",
        "your response", "write your response here", "replace this text",
    }
    normalized = response.lower().strip()
    normalized = re.sub(r"[^a-z0-9\s/.-]", "", normalized)
    normalized = re.sub(r"\s+", " ", normalized).strip()

    if not response:
        raise ReflectionError(_reflection_message(
            qid, filename, "The block is present, but YOUR RESPONSE is blank."
        ))
    if normalized in placeholders:
        raise ReflectionError(_reflection_message(
            qid, filename, "The response appears to be placeholder text.", found=response
        ))

    words = re.findall(r"[A-Za-z0-9']+", response)
    if len(words) < REFLECTION_MIN_WORDS:
        raise ReflectionError(_reflection_message(
            qid, filename,
            f"The response is too short to count as a complete reflection. It has {len(words)} words; expected at least {REFLECTION_MIN_WORDS}.",
            found=response,
        ))
    if not has_genai_disclosure(response):
        raise ReflectionError(_reflection_message(
            qid, filename,
            "The response does not clearly say whether you used GenAI, artificial intelligence, machine learning, a coding assistant, or no such tool for this question.",
            found=response,
        ))
    if not has_concrete_project_detail(response):
        raise ReflectionError(_reflection_message(
            qid, filename,
            "The response is too generic. Include at least one concrete project detail such as a function name, file name, map name, test id, or command.",
            found=response,
        ))

    return f"Completed self-reflection in {filename} ({len(words)} words)."

def reflection_case(qid):
    filename = REFLECTION_LOCATIONS[qid]
    return make_case(f"{qid.upper()}.R Self-reflection for {qid.upper()}", REFLECTION_POINTS,
                     lambda qid=qid, filename=filename: check_reflection(qid, filename))

def run_plan(objective, plan):
    state = objective.initial_state()
    for action in plan:
        choices = {t.action: t for t in objective.successors(state)}
        if action not in choices:
            raise AssertionError(f"Illegal action {action!r} from state {state}.")
        state = choices[action].next_state
    return state

def plan_cost(objective, plan):
    state = objective.initial_state()
    total = 0
    for action in plan:
        choices = {t.action: t for t in objective.successors(state)}
        if action not in choices:
            raise AssertionError(f"Illegal action {action!r} from state {state}.")
        t = choices[action]
        total += t.cost
        state = t.next_state
    return total, state

def check_consistent_heuristic(objective, heuristic, label):
    start = objective.initial_state()
    start_h = heuristic(start, objective)
    assert start_h > 0, f"{label} heuristic should be positive at the start state; got {start_h}."

    frontier = deque([start])
    seen = {start}
    edges_checked = 0

    while frontier:
        state = frontier.popleft()
        h_state = heuristic(state, objective)
        assert h_state >= 0, f"{label} heuristic returned a negative value {h_state} at state {state}."
        if objective.is_goal(state):
            assert h_state == 0, f"{label} heuristic should be 0 at goal states; got {h_state} at {state}."

        for transition in objective.successors(state):
            next_h = heuristic(transition.next_state, objective)
            edges_checked += 1
            assert h_state <= transition.cost + next_h + 1e-9, (
                f"{label} heuristic is inconsistent for action {transition.action!r}: "
                f"h(state)={h_state}, cost={transition.cost}, h(next)={next_h}, "
                f"state={state}, next={transition.next_state}."
            )
            if transition.next_state not in seen:
                seen.add(transition.next_state)
                frontier.append(transition.next_state)

    return start_h, len(seen), edges_checked

_CONSISTENCY_CACHE = {}

def cached_consistency(key, objective, heuristic, label):
    cache_key = (key, id(heuristic))
    if cache_key not in _CONSISTENCY_CACHE:
        _CONSISTENCY_CACHE[cache_key] = check_consistent_heuristic(objective, heuristic, label)
    return _CONSISTENCY_CACHE[cache_key]

_HEURISTIC_SANITY_CACHE = {}

def cached_rescue_heuristic_sanity(key, objective, heuristic):
    """Run direct checks that catch overestimates even if graph traversal changes."""
    cache_key = (key, id(heuristic))
    if cache_key in _HEURISTIC_SANITY_CACHE:
        return _HEURISTIC_SANITY_CACHE[cache_key]

    from planner import uniform_cost_plan
    from rescue_core.mission import MissionState

    start = objective.initial_state()
    start_h = heuristic(start, objective)
    optimal_plan = uniform_cost_plan(objective)
    optimal_cost, final = plan_cost(objective, optimal_plan)
    assert objective.is_goal(final), "UCS sanity check did not reach a rescue-all goal."
    assert start_h <= optimal_cost + 1e-9, (
        "remaining_survivor_heuristic overestimates the true optimal rescue-all cost on "
        f"survivor_cluster: h(start)={start_h}, optimal_cost={optimal_cost}. "
        "A hardcoded large value such as 9999 should fail here."
    )

    goal_checks = 0
    for loc in objective.locations.values():
        goal_state = MissionState(loc, frozenset(), objective.mission.initial_battery)
        goal_h = heuristic(goal_state, objective)
        assert goal_h == 0, (
            "remaining_survivor_heuristic must return 0 on rescue-all goal states; "
            f"got {goal_h} at {goal_state}."
        )
        goal_checks += 1

    _HEURISTIC_SANITY_CACHE[cache_key] = (start_h, optimal_cost, goal_checks)
    return _HEURISTIC_SANITY_CACHE[cache_key]

class CountingObjective:
    def __init__(self, objective):
        self.objective = objective
        self.expanded = 0

    def initial_state(self):
        return self.objective.initial_state()

    def is_goal(self, state):
        return self.objective.is_goal(state)

    def successors(self, state):
        self.expanded += 1
        return self.objective.successors(state)

    def __getattr__(self, name):
        return getattr(self.objective, name)

def astar_expansion_metrics(objective, heuristic):
    from planner import astar_plan
    counted = CountingObjective(objective)
    plan = astar_plan(counted, heuristic)
    cost, final = plan_cost(objective, plan)
    assert objective.is_goal(final), f"A* did not reach the goal. Plan: {plan}, cost: {cost}"
    return counted.expanded, cost, plan

def make_case(name, max_score, fn):
    return {"name": name, "max_score": max_score, "fn": fn}

def run_case(case):
    try:
        out = case["fn"]()
        return result(case["name"], case["max_score"], case["max_score"], out or "Passed.")
    except ReflectionError as exc:
        return result(case["name"], 0, case["max_score"], str(exc))
    except Exception as exc:
        expected, check = CASE_HINTS.get(case["name"], ("See the test name for the expected behavior.", "Check the function named by this question."))
        output = f"{type(exc).__name__}: {exc}\n" + traceback.format_exc(limit=4)
        output = f"Expected behavior: {expected}\nLikely place to check: {check}\n\n{output}"
        return result(case["name"], 0, case["max_score"], output)

def q1_cases():
    def stack_queue_basics():
        from planner import SearchNode, StackFrontier
        s = StackFrontier()
        s.push(SearchNode(0, "first", [], 0))
        s.push(SearchNode(0, "second", [], 0))
        assert s.pop().state == "second", "StackFrontier must be last-in, first-out."
        from planner import SearchNode, QueueFrontier
        q = QueueFrontier()
        q.push(SearchNode(0, "first", [], 0))
        q.push(SearchNode(0, "second", [], 0))
        assert q.pop().state == "first", "QueueFrontier must be first-in, first-out."
        return "StackFrontier is LIFO and QueueFrontier is FIFO."

    def priority_min_first():
        from planner import SearchNode, PriorityFrontier
        p = PriorityFrontier()
        p.push(SearchNode(5, "expensive", [], 0))
        p.push(SearchNode(1, "cheap", [], 0))
        assert p.pop().state == "cheap", "PriorityFrontier must pop lowest priority first."
        return "PriorityFrontier popped the lowest-priority state."

    def graph_search_cost_priority():
        from planner import PriorityFrontier, graph_search
        from rescue_core.mission import Transition
        class ToyObjective:
            def initial_state(self): return "A"
            def is_goal(self, state): return state == "D"
            def successors(self, state):
                edges = {
                    "A": [("toB", "B", 2), ("toC", "C", 1)],
                    "B": [("toD", "D", 2)],
                    "C": [("toD", "D", 10)],
                    "D": [],
                }
                return [Transition(a, nxt, c) for a, nxt, c in edges[state]]
        plan = graph_search(ToyObjective(), PriorityFrontier(), lambda node: node.path_cost)
        assert plan == ["toB", "toD"], f"Cost-priority graph_search returned {plan}."
        return "graph_search selected the lower-cost route in a toy graph."

    def dominated_battery_states():
        from planner import StackFrontier, graph_search
        from rescue_core.location import GridLocation
        from rescue_core.mission import MissionState, Transition
        class BatteryLoopObjective:
            def initial_state(self):
                return MissionState(GridLocation(0, 0), frozenset(["T"]), 5)
            def is_goal(self, state):
                return state.robot == GridLocation(0, 2)
            def successors(self, state):
                loc = state.robot
                remaining = state.remaining
                battery = state.battery
                if loc == GridLocation(0, 0):
                    return [Transition("E", MissionState(GridLocation(0, 1), remaining, battery - 1), 1)]
                if loc == GridLocation(0, 1):
                    return [
                        Transition("E", MissionState(GridLocation(0, 2), frozenset(), battery - 1), 1),
                        Transition("W", MissionState(GridLocation(0, 0), remaining, battery - 1), 1),
                    ]
                return []
        plan = graph_search(BatteryLoopObjective(), StackFrontier())
        assert plan == ["E", "E"], (
            "graph_search should not revisit the same mission-progress state with less battery; "
            f"got {plan}."
        )
        return "graph_search ignored dominated lower-battery states."

    return [
        make_case("Q1.1 Stack and Queue frontiers behave correctly", 0.75, stack_queue_basics),
        make_case("Q1.2 PriorityFrontier uses lowest priority first", 0.75, priority_min_first),
        make_case("Q1.3 graph_search works with cost priority", 1, graph_search_cost_priority),
        make_case("Q1.4 graph_search handles dominated battery states", 1, dominated_battery_states),
        reflection_case("q1"),
    ]

def q2_cases():
    def bfs_tiny_shortest():
        from planner import breadth_first_plan
        from objectives import ReachTargetObjective
        mission = load_mission(resolve_map_path("tiny_rescue"))
        obj = ReachTargetObjective(mission, "S1")
        bfs = breadth_first_plan(obj)
        assert bfs == ["E", "E", "E", "E"], f"BFS should find the 4-step plan on tiny_rescue; got {bfs}."
        return f"BFS returned the expected shortest-step plan: {bfs}"

    def dfs_reaches_survivor():
        from planner import depth_first_plan
        from objectives import ReachTargetObjective
        mission = load_mission(resolve_map_path("tiny_rescue"))
        obj = ReachTargetObjective(mission, "S1")
        dfs = depth_first_plan(obj)
        final = run_plan(obj, dfs)
        assert obj.is_goal(final), f"DFS plan did not reach the survivor. Plan: {dfs}"
        return f"DFS reached the target survivor with plan length {len(dfs)}."

    return [
        make_case("Q2.1 BFS finds shortest plan on tiny_rescue", 1.75, bfs_tiny_shortest),
        make_case("Q2.2 DFS returns a valid survivor-reaching plan", 1.75, dfs_reaches_survivor),
        reflection_case("q2"),
    ]

def q3_cases():
    def ucs_rubble_corridor():
        from planner import uniform_cost_plan
        from objectives import ReachTargetObjective
        mission = load_mission(resolve_map_path("rubble_corridor"))
        obj = ReachTargetObjective(mission, "S1")
        plan = uniform_cost_plan(obj)
        cost, final = plan_cost(obj, plan)
        assert obj.is_goal(final), "UCS plan did not reach the survivor."
        assert cost == 8, f"UCS should achieve cost 8 on rubble_corridor; got cost {cost} with plan {plan}."
        return f"UCS reached the survivor on rubble_corridor with optimal cost {cost}."

    def ucs_battery_challenge():
        from planner import uniform_cost_plan
        from objectives import ReachTargetObjective
        mission = load_mission(resolve_map_path("battery_challenge"))
        obj = ReachTargetObjective(mission, "S1")
        plan = uniform_cost_plan(obj)
        cost, final = plan_cost(obj, plan)
        assert obj.is_goal(final), "UCS should reach the survivor on battery_challenge."
        visited_charger = any(mission.is_charger(frame[0]) for frame in replay_plan(mission, plan))
        assert visited_charger, (
            "On battery_challenge, UCS should choose the longer lower-cost route through the charging station; "
            f"got plan {plan} with cost {cost}."
        )
        assert cost == 30, (
            "UCS should avoid the shorter but more expensive corridors and achieve cost 30 on battery_challenge; "
            f"got cost {cost} with plan {plan}."
        )
        assert final.battery >= 0, "The plan should finish without depleting the battery."
        return f"UCS used the lower charger route on battery_challenge with cost {cost} and battery {final.battery}."

    return [
        make_case("Q3.1 UCS chooses lower-cost route on rubble_corridor", 1.75, ucs_rubble_corridor),
        make_case("Q3.2 UCS uses charger route on battery_challenge", 1.75, ucs_battery_challenge),
        reflection_case("q3"),
    ]

def q4_cases():
    def astar_reaches_target():
        from planner import astar_plan
        from objectives import ReachTargetObjective
        from heuristics import distance_to_target_heuristic
        mission = load_mission(resolve_map_path("fire_escape"))
        obj = ReachTargetObjective(mission, "S1")
        plan = astar_plan(obj, distance_to_target_heuristic)
        cost, final = plan_cost(obj, plan)
        assert obj.is_goal(final), f"A* plan did not reach the survivor. Plan: {plan}, cost: {cost}"
        return f"A* reached the target on fire_escape with cost {cost}."

    def astar_matches_ucs_cost():
        from planner import uniform_cost_plan, astar_plan
        from objectives import ReachTargetObjective
        from heuristics import distance_to_target_heuristic
        mission = load_mission(resolve_map_path("fire_escape"))
        obj = ReachTargetObjective(mission, "S1")
        ucs_plan = uniform_cost_plan(obj)
        astar = astar_plan(obj, distance_to_target_heuristic)
        ucs_cost, _ = plan_cost(obj, ucs_plan)
        astar_cost, _ = plan_cost(obj, astar)
        assert astar_cost == ucs_cost, f"A* cost {astar_cost} should match UCS optimal cost {ucs_cost}."
        return f"A* matched UCS optimal cost {ucs_cost}."

    return [
        make_case("Q4.1 A* reaches the survivor on fire_escape", 1.75, astar_reaches_target),
        make_case("Q4.2 A* matches UCS optimal cost", 1.75, astar_matches_ucs_cost),
        reflection_case("q4"),
    ]

def q5_cases():
    def inspection_completes_all_beacons():
        from planner import uniform_cost_plan
        from objectives import InspectLocationsObjective
        mission = load_mission(resolve_map_path("beacon_rooms"))
        obj = InspectLocationsObjective(mission)
        plan = uniform_cost_plan(obj)
        cost, final = plan_cost(obj, plan)
        assert obj.is_goal(final), "Inspection objective did not complete all beacons."
        return f"Inspection objective completed all beacons with cost {cost}."

    def inspection_remaining_empty():
        from planner import uniform_cost_plan
        from objectives import InspectLocationsObjective
        mission = load_mission(resolve_map_path("beacon_rooms"))
        obj = InspectLocationsObjective(mission)
        plan = uniform_cost_plan(obj)
        _, final = plan_cost(obj, plan)
        assert final.remaining == frozenset(), "All beacon labels should be removed from the state."
        return "Inspection state removed all completed beacon labels."

    def inspection_cost_reasonable():
        from planner import uniform_cost_plan
        from objectives import InspectLocationsObjective
        mission = load_mission(resolve_map_path("beacon_rooms"))
        obj = InspectLocationsObjective(mission)
        plan = uniform_cost_plan(obj)
        cost, _ = plan_cost(obj, plan)
        assert cost <= 34, f"Inspection route is unexpectedly expensive: cost {cost}, plan {plan}."
        return f"Inspection route cost {cost} is within the expected bound."

    return [
        make_case("Q5.1 Inspection objective reaches all beacons", 1.5, inspection_completes_all_beacons),
        make_case("Q5.2 Inspection state tracks remaining labels", 1, inspection_remaining_empty),
        make_case("Q5.3 Inspection route cost is reasonable", 1, inspection_cost_reasonable),
        reflection_case("q5"),
    ]

def q6_cases():
    def inspection_heuristic_consistent():
        from objectives import InspectLocationsObjective
        from heuristics import remaining_inspection_heuristic
        mission = load_mission(resolve_map_path("beacon_rooms"))
        obj = InspectLocationsObjective(mission)
        h, states, edges = cached_consistency("inspection", obj, remaining_inspection_heuristic, "Inspection")
        return f"Inspection heuristic start value {h}; checked consistency over {states} states and {edges} transitions."

    def survivor_heuristic_consistent():
        from objectives import RescueAllSurvivorsObjective
        from heuristics import remaining_survivor_heuristic
        mission = load_mission(resolve_map_path("survivor_cluster"))
        obj = RescueAllSurvivorsObjective(mission)
        start_h, optimal_cost, goal_checks = cached_rescue_heuristic_sanity(
            "rescue_all_sanity", obj, remaining_survivor_heuristic
        )
        h, states, edges = cached_consistency("rescue_all", obj, remaining_survivor_heuristic, "Rescue-all")
        return (
            f"Rescue-all heuristic start value {h}; optimal cost sanity bound {optimal_cost}; "
            f"checked {goal_checks} direct goal states, {states} reachable states, and {edges} transitions."
        )

    def survivor_heuristic_beats_zero():
        from objectives import RescueAllSurvivorsObjective
        from heuristics import remaining_survivor_heuristic
        small = RescueAllSurvivorsObjective(load_mission(resolve_map_path("survivor_cluster")))
        cached_rescue_heuristic_sanity("rescue_all_sanity", small, remaining_survivor_heuristic)
        cached_consistency("rescue_all", small, remaining_survivor_heuristic, "Rescue-all")
        mission = load_mission(resolve_map_path("large_survivor_cluster"))
        obj = RescueAllSurvivorsObjective(mission)
        zero_expanded, zero_cost, _ = astar_expansion_metrics(obj, lambda state, objective: 0)
        h_expanded, h_cost, _ = astar_expansion_metrics(obj, remaining_survivor_heuristic)
        assert h_cost == zero_cost, f"Heuristic A* cost {h_cost} should match zero-heuristic optimal cost {zero_cost}."
        threshold = zero_expanded * 0.65
        assert h_expanded <= threshold, (
            f"Heuristic expanded {h_expanded} nodes; expected <= 65% of zero-heuristic expansions "
            f"({zero_expanded}), threshold {threshold:.1f}."
        )
        return f"Expanded {h_expanded} nodes versus zero heuristic {zero_expanded}; cost {h_cost}."

    def survivor_heuristic_strong_tier():
        from objectives import RescueAllSurvivorsObjective
        from heuristics import remaining_survivor_heuristic
        small = RescueAllSurvivorsObjective(load_mission(resolve_map_path("survivor_cluster")))
        cached_rescue_heuristic_sanity("rescue_all_sanity", small, remaining_survivor_heuristic)
        cached_consistency("rescue_all", small, remaining_survivor_heuristic, "Rescue-all")
        mission = load_mission(resolve_map_path("large_survivor_cluster"))
        obj = RescueAllSurvivorsObjective(mission)
        zero_expanded, zero_cost, _ = astar_expansion_metrics(obj, lambda state, objective: 0)
        h_expanded, h_cost, _ = astar_expansion_metrics(obj, remaining_survivor_heuristic)
        assert h_cost == zero_cost, f"Heuristic A* cost {h_cost} should match zero-heuristic optimal cost {zero_cost}."
        threshold = zero_expanded * 0.35
        assert h_expanded <= threshold, (
            f"Heuristic expanded {h_expanded} nodes; expected <= 35% of zero-heuristic expansions "
            f"({zero_expanded}), threshold {threshold:.1f}."
        )
        return f"Strong tier reached: expanded {h_expanded} nodes versus zero heuristic {zero_expanded}; cost {h_cost}."

    return [
        make_case("Q6.1 Inspection heuristic is consistent", 0.75, inspection_heuristic_consistent),
        make_case("Q6.2 Rescue-all heuristic is consistent", 0.75, survivor_heuristic_consistent),
        make_case("Q6.3 Rescue-all heuristic beats UCS expansions", 1, survivor_heuristic_beats_zero),
        make_case("Q6.4 Rescue-all heuristic earns strong expansion score", 1, survivor_heuristic_strong_tier),
        reflection_case("q6"),
    ]

def q7_cases():
    def greedy_rescues_all():
        from planner import GreedyRescuePlanner
        from objectives import RescueAllSurvivorsObjective
        from heuristics import rescue_priority_score
        mission = load_mission(resolve_map_path("survivor_cluster"))
        obj = RescueAllSurvivorsObjective(mission)
        plan = GreedyRescuePlanner(rescue_priority_score).plan(obj)
        cost, final = plan_cost(obj, plan)
        assert obj.is_goal(final), f"GreedyRescuePlanner did not rescue all survivors. Plan: {plan}, cost: {cost}"
        return f"Greedy planner rescued all survivors with cost {cost}."

    def greedy_feasible_budgeted_plan():
        from planner import GreedyRescuePlanner
        from objectives import RescueAllSurvivorsObjective
        from heuristics import rescue_priority_score
        mission = load_mission(resolve_map_path("survivor_cluster"))
        obj = RescueAllSurvivorsObjective(mission)
        plan = GreedyRescuePlanner(rescue_priority_score).plan(obj)
        cost, final = plan_cost(obj, plan)
        assert final.battery >= 0, "GreedyRescuePlanner returned a plan that depletes the battery."
        assert cost <= mission.initial_battery, f"Plan cost {cost} must not exceed initial battery {mission.initial_battery}."
        return f"Greedy planner finished with battery {final.battery} and cost {cost}."

    return [
        make_case("Q7.1 Greedy planner rescues all survivors", 0.75, greedy_rescues_all),
        make_case("Q7.2 Greedy planner returns a feasible budgeted plan", 0.75, greedy_feasible_budgeted_plan),
        reflection_case("q7"),
    ]

_Q8_CACHE = None

def q8_metrics():
    global _Q8_CACHE
    if _Q8_CACHE is not None:
        return _Q8_CACHE

    from planner import SimulatedAnnealingRescuePlanner
    from objectives import RescueAllSurvivorsObjective

    mission = load_mission(resolve_map_path("route_optimization"))
    obj = RescueAllSurvivorsObjective(mission)
    planner_a = SimulatedAnnealingRescuePlanner(seed=7, iterations=300)
    planner_b = SimulatedAnnealingRescuePlanner(seed=7, iterations=300)
    plan_a = planner_a.plan(obj)
    plan_b = planner_b.plan(obj)
    cost_a, final_a = plan_cost(obj, plan_a)
    cost_b, final_b = plan_cost(obj, plan_b)
    _Q8_CACHE = {
        "mission": mission,
        "objective": obj,
        "plan_a": plan_a,
        "plan_b": plan_b,
        "cost_a": cost_a,
        "cost_b": cost_b,
        "final_a": final_a,
        "final_b": final_b,
    }
    return _Q8_CACHE

def q8_cases():
    def anneal_rescues_all():
        metrics = q8_metrics()
        obj = metrics["objective"]
        assert obj.is_goal(metrics["final_a"]), (
            f"Annealing plan did not rescue all survivors. Plan: {metrics['plan_a']}, "
            f"cost: {metrics['cost_a']}."
        )
        return f"Annealing planner rescued all survivors with cost {metrics['cost_a']}."

    def anneal_deterministic():
        metrics = q8_metrics()
        assert metrics["plan_a"] == metrics["plan_b"], (
            "Two annealing runs with the same seed should return the same plan. "
            f"First: {metrics['plan_a']}; second: {metrics['plan_b']}."
        )
        assert metrics["cost_a"] == metrics["cost_b"], "Seeded annealing runs should have the same cost."
        return "Annealing planner is deterministic for seed=7."

    def anneal_beats_greedy_baseline():
        metrics = q8_metrics()
        assert metrics["cost_a"] <= 48, (
            "The route_optimization greedy baseline costs 52. "
            f"Expected annealing cost <= 48; got {metrics['cost_a']}."
        )
        return f"Annealing improved over greedy baseline with cost {metrics['cost_a']}."

    def anneal_strong_target():
        metrics = q8_metrics()
        assert metrics["cost_a"] <= 40, (
            f"Expected annealing cost <= 40 on route_optimization; got {metrics['cost_a']}."
        )
        return f"Annealing reached the strong optimization target with cost {metrics['cost_a']}."

    return [
        make_case("Q8.1 Annealing planner rescues all survivors", 0.75, anneal_rescues_all),
        make_case("Q8.2 Annealing planner is deterministic with a seed", 0.75, anneal_deterministic),
        make_case("Q8.3 Annealing improves over greedy baseline", 1, anneal_beats_greedy_baseline),
        make_case("Q8.4 Annealing reaches strong optimization target", 1, anneal_strong_target),
        reflection_case("q8"),
    ]

QUESTION_CASES = [
    ("q1", "Q1 Frontiers and graph search", 4, q1_cases),
    ("q2", "Q2 DFS and BFS rescue planning", 4, q2_cases),
    ("q3", "Q3 Terrain-aware uniform-cost planning", 4, q3_cases),
    ("q4", "Q4 A* rescue planning", 4, q4_cases),
    ("q5", "Q5 Multi-location inspection objective", 4, q5_cases),
    ("q6", "Q6 Consistent and informative heuristics", 4, q6_cases),
    ("q7", "Q7 Greedy multi-survivor planning", 2, q7_cases),
    ("q8", "Q8 Simulated annealing rescue optimization", 4, q8_cases),
]

def _select_questions(question_id):
    if question_id is None:
        return QUESTION_CASES
    q = question_id.lower().strip()
    lookup = {qid: item for qid, *item in QUESTION_CASES}
    if q not in lookup:
        valid = ", ".join(qid for qid, *_ in QUESTION_CASES)
        raise SystemExit(f"Unknown question {question_id!r}. Valid choices: {valid}")
    title, max_score, cases_fn = lookup[q]
    return [(q, title, max_score, cases_fn)]

def main(argv=None):
    parser = argparse.ArgumentParser(description="Rescue Robot Project 1 autograder")
    parser.add_argument("-q", "--question", dest="question", default=None,
                        help="Run only one question, for example: -q q1")
    parser.add_argument("--no-graphics", action="store_true",
                        help="Accepted for compatibility; the autograder never opens graphics.")
    args = parser.parse_args(argv)

    selected = _select_questions(args.question)
    all_results = []
    total = 0
    max_total = 0

    for qid, title, qmax, cases_fn in selected:
        print(f"\n{title}")
        print("-" * len(title))
        q_results = []
        for case in cases_fn():
            r = run_case(case)
            q_results.append(r)
            all_results.append(r)
            print(f"{r['name']}: {format_points(r['score'])}/{format_points(r['max_score'])} {'PASS' if r['status']=='passed' else 'FAIL'}")
            if r["status"] != "passed":
                print(r["output"].rstrip())
        q_score = sum(r["score"] for r in q_results)
        q_max = sum(r["max_score"] for r in q_results)
        total += q_score
        max_total += q_max
        print(f"{title} subtotal: {format_points(q_score)}/{format_points(q_max)}")

    print(f"\nTotal: {format_points(total)}/{format_points(max_total)}")
    payload = {
        "score": total,
        "max_score": max_total,
        "tests": all_results,
        "stdout_visibility": "visible",
    }
    out_dir = Path("/autograder/results")
    if out_dir.exists():
        (out_dir / "results.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    else:
        Path("results.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return 0 if total == max_total else 1

if __name__ == "__main__":
    raise SystemExit(main())
