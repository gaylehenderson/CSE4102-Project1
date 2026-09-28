from __future__ import annotations

import math
import time

TILE = 48
PADDING = 24
HUD = 170
MIN_WINDOW_WIDTH = 900

COLORS = {
    "bg": (17, 24, 32),
    "panel": (24, 34, 45),
    "grid": (53, 67, 83),
    "wall": (43, 48, 55),
    "wall_edge": (31, 37, 45),
    "clear": (224, 232, 238),
    "clear_alt": (216, 226, 233),
    "rubble": (168, 160, 137),
    "rubble_dark": (78, 73, 63),
    "rubble_light": (211, 202, 174),
    "sand": (226, 185, 102),
    "sand_dark": (58, 41, 26),
    "sand_light": (255, 218, 147),
    "sand_mid": (241, 199, 111),
    "sand_shadow": (117, 80, 39),
    "water": (78, 147, 194),
    "water_light": (155, 210, 236),
    "fire": (220, 87, 56),
    "fire_light": (255, 190, 70),
    "charge": (77, 180, 126),
    "charge_light": (177, 247, 207),
    "robot_yellow": (247, 190, 56),
    "robot_yellow_dark": (195, 134, 32),
    "robot_body": (244, 202, 97),
    "robot_panel": (210, 143, 48),
    "robot_tread": (44, 48, 53),
    "robot_tread_light": (91, 96, 103),
    "robot_eye": (245, 249, 255),
    "robot_pupil": (28, 38, 50),
    "robot_glint": (255, 255, 255),
    "survivor": (255, 211, 106),
    "survivor_done": (92, 174, 113),
    "survivor_clothes": (89, 149, 223),
    "beacon": (152, 112, 255),
    "beacon_core": (207, 187, 255),
    "beacon_done": (92, 198, 255),
    "route": (61, 169, 255),
    "route_dark": (26, 112, 187),
    "text": (240, 243, 247),
    "muted": (179, 188, 198),
    "battery_bg": (61, 70, 82),
    "battery": (91, 214, 120),
    "battery_mid": (245, 194, 66),
    "battery_low": (238, 92, 92),
}

def _require_pygame():
    try:
        import pygame
    except Exception as exc:
        raise RuntimeError(
            "pygame is required for graphics. Install it with: "
            "python3.13 -m pip install pygame"
        ) from exc
    return pygame

def _rect_for(loc):
    return (PADDING + loc.col * TILE, PADDING + loc.row * TILE, TILE, TILE)

def _center_for(loc):
    x, y, w, h = _rect_for(loc)
    return x + w // 2, y + h // 2

def _window_size(mission):
    # Use a minimum width so the status panel and legend remain readable
    # even for small mission maps.
    grid_width = PADDING * 2 + mission.width * TILE
    width = max(MIN_WINDOW_WIDTH, grid_width)
    height = PADDING * 2 + mission.height * TILE + HUD
    return width, height

def _tile_color(ch, row=0, col=0):
    if ch == "#":
        return COLORS["wall"]
    if ch == "r":
        return COLORS["rubble"]
    if ch == "w":
        return COLORS["water"]
    if ch == "f":
        return COLORS["fire"]
    if ch == "m":
        return COLORS["sand"]
    if ch == "c":
        return COLORS["charge"]
    return COLORS["clear"] if (row + col) % 2 == 0 else COLORS["clear_alt"]

def _draw_tile_details(pygame, surface, ch, rect):
    x, y, w, h = rect
    if ch == "#":
        pygame.draw.rect(surface, COLORS["wall_edge"], rect, 2, border_radius=4)
        pygame.draw.line(surface, (58, 66, 76), (x + 8, y + 12), (x + w - 10, y + 12), 2)
        pygame.draw.line(surface, (32, 37, 44), (x + 10, y + h - 10), (x + w - 8, y + h - 10), 2)
    elif ch == "r":
        # Stacked stone rubble pile, distinct from the sand dunes.
        pygame.draw.ellipse(surface, COLORS["rubble_dark"], (x + 6, y + 36, 37, 7))
        stones = [
            ([(x + 8, y + 35), (x + 13, y + 22), (x + 22, y + 20), (x + 27, y + 36), (x + 16, y + 41)], COLORS["rubble_light"]),
            ([(x + 23, y + 36), (x + 31, y + 23), (x + 42, y + 26), (x + 39, y + 41), (x + 27, y + 41)], COLORS["rubble"]),
            ([(x + 16, y + 24), (x + 24, y + 10), (x + 35, y + 16), (x + 31, y + 30), (x + 20, y + 31)], COLORS["rubble_light"]),
            ([(x + 27, y + 23), (x + 37, y + 13), (x + 44, y + 22), (x + 38, y + 32), (x + 29, y + 30)], COLORS["rubble"]),
            ([(x + 11, y + 40), (x + 22, y + 34), (x + 31, y + 40), (x + 24, y + 45), (x + 12, y + 45)], (141, 132, 109)),
        ]
        for points, color in stones:
            shadow = [(px + 2, py + 2) for px, py in points]
            pygame.draw.polygon(surface, COLORS["rubble_dark"], shadow)
            pygame.draw.polygon(surface, color, points)
            pygame.draw.polygon(surface, COLORS["rubble_dark"], points, 2)
        crack_color = (104, 96, 82)
        pygame.draw.line(surface, crack_color, (x + 17, y + 25), (x + 22, y + 32), 2)
        pygame.draw.line(surface, crack_color, (x + 25, y + 14), (x + 31, y + 21), 2)
        pygame.draw.line(surface, crack_color, (x + 31, y + 27), (x + 38, y + 29), 2)
        pygame.draw.line(surface, crack_color, (x + 17, y + 39), (x + 23, y + 36), 2)
    elif ch == "m":
        # Sand dunes: rolling hills with small pebbles, not a brown mound.
        pygame.draw.ellipse(surface, COLORS["sand_shadow"], (x + 5, y + 39, 38, 6))
        rear_dune = [
            (x + 9, y + 31), (x + 15, y + 22), (x + 20, y + 21),
            (x + 25, y + 27), (x + 30, y + 25), (x + 36, y + 15),
            (x + 43, y + 18), (x + 46, y + 31), (x + 46, y + 42),
            (x + 9, y + 42),
        ]
        front_dune = [
            (x + 2, y + 39), (x + 10, y + 31), (x + 18, y + 29),
            (x + 27, y + 35), (x + 35, y + 28), (x + 46, y + 37),
            (x + 46, y + 44), (x + 2, y + 44),
        ]
        pygame.draw.polygon(surface, COLORS["sand_mid"], rear_dune)
        pygame.draw.lines(surface, COLORS["sand_dark"], False, rear_dune[:8], 3)
        pygame.draw.polygon(surface, COLORS["sand_light"], [(x + 16, y + 39), (x + 25, y + 28), (x + 35, y + 39)])
        pygame.draw.polygon(surface, COLORS["sand_mid"], front_dune)
        pygame.draw.lines(surface, COLORS["sand_dark"], False, front_dune[:6], 3)
        pygame.draw.line(surface, COLORS["sand_dark"], (x + 11, y + 38), (x + 22, y + 34), 2)
        pygame.draw.line(surface, COLORS["sand_dark"], (x + 29, y + 37), (x + 40, y + 35), 2)
        for px, py, radius in [(9, 38, 2), (18, 34, 2), (28, 31, 2), (39, 37, 2)]:
            pygame.draw.circle(surface, COLORS["sand_dark"], (x + px, y + py), radius)
    elif ch == "w":
        for i in range(3):
            yy = y + 16 + i * 8
            pygame.draw.arc(surface, COLORS["water_light"], (x + 8, yy, 16, 8), 0, math.pi, 2)
            pygame.draw.arc(surface, COLORS["water_light"], (x + 23, yy, 16, 8), 0, math.pi, 2)
    elif ch == "f":
        cx, cy = x + w // 2, y + h // 2
        pygame.draw.polygon(surface, COLORS["fire_light"], [(cx, cy - 16), (cx - 12, cy + 14), (cx + 13, cy + 14)])
        pygame.draw.polygon(surface, COLORS["fire"], [(cx + 5, cy - 9), (cx - 8, cy + 13), (cx + 10, cy + 13)])
        pygame.draw.polygon(surface, (255, 236, 133), [(cx - 2, cy - 4), (cx - 5, cy + 11), (cx + 4, cy + 11)])
    elif ch == "c":
        pygame.draw.circle(surface, COLORS["charge_light"], (x + w//2, y + h//2), 14)
        pygame.draw.circle(surface, COLORS["charge"], (x + w//2, y + h//2), 16, 3)
        bolt = [(x+24, y+11), (x+17, y+25), (x+24, y+24), (x+20, y+37), (x+32, y+20), (x+25, y+21)]
        pygame.draw.polygon(surface, (255, 245, 135), bolt)

def _draw_route_marker(pygame, surface, loc, prev_loc=None, next_loc=None):
    x, y, w, h = _rect_for(loc)
    cx, cy = x + w // 2, y + h // 2
    # A small blue navigation arrow is clearer than collectible-looking dots.
    if prev_loc and next_loc:
        dr = next_loc.row - prev_loc.row
        dc = next_loc.col - prev_loc.col
    elif next_loc:
        dr = next_loc.row - loc.row
        dc = next_loc.col - loc.col
    elif prev_loc:
        dr = loc.row - prev_loc.row
        dc = loc.col - prev_loc.col
    else:
        dr, dc = 0, 1
    if abs(dc) >= abs(dr):
        if dc >= 0:
            pts = [(cx + 9, cy), (cx - 5, cy - 7), (cx - 5, cy + 7)]
        else:
            pts = [(cx - 9, cy), (cx + 5, cy - 7), (cx + 5, cy + 7)]
    else:
        if dr >= 0:
            pts = [(cx, cy + 9), (cx - 7, cy - 5), (cx + 7, cy - 5)]
        else:
            pts = [(cx, cy - 9), (cx - 7, cy + 5), (cx + 7, cy + 5)]
    pygame.draw.circle(surface, (218, 239, 255), (cx, cy), 10)
    pygame.draw.polygon(surface, COLORS["route"], pts)
    pygame.draw.circle(surface, COLORS["route_dark"], (cx, cy), 10, 1)

def _draw_robot(pygame, surface, loc):
    """Draw an original cute yellow tracked rescue robot.

    The design is intentionally a generic friendly rescue bot: yellow body,
    big binocular eyes, small treads, and a rescue light. It is not a copied
    character asset.
    """
    x, y, w, h = _rect_for(loc)
    cx = x + w // 2

    # Tracks
    pygame.draw.rect(surface, COLORS["robot_tread"], (x + 5, y + 28, 10, 14), border_radius=5)
    pygame.draw.rect(surface, COLORS["robot_tread"], (x + w - 15, y + 28, 10, 14), border_radius=5)
    for off in (8, 13, 32, 37):
        pygame.draw.circle(surface, COLORS["robot_tread_light"], (x + off, y + 35), 2)

    # Body
    body = pygame.Rect(x + 12, y + 23, w - 24, 18)
    pygame.draw.rect(surface, COLORS["robot_yellow_dark"], body.move(0, 2), border_radius=6)
    pygame.draw.rect(surface, COLORS["robot_body"], body, border_radius=6)
    pygame.draw.rect(surface, COLORS["robot_panel"], (x + 16, y + 27, 10, 8), border_radius=2)
    pygame.draw.rect(surface, (126, 198, 102), (x + 28, y + 28, 5, 3), border_radius=1)
    pygame.draw.rect(surface, (236, 88, 72), (x + 35, y + 28, 5, 3), border_radius=1)

    # Neck and head
    pygame.draw.rect(surface, COLORS["robot_yellow_dark"], (cx - 3, y + 18, 6, 7), border_radius=2)
    pygame.draw.line(surface, COLORS["robot_tread"], (cx - 7, y + 15), (cx - 15, y + 10), 2)
    pygame.draw.line(surface, COLORS["robot_tread"], (cx + 7, y + 15), (cx + 15, y + 10), 2)

    # Big binocular eyes
    left_eye = (cx - 9, y + 14)
    right_eye = (cx + 9, y + 14)
    for ex, ey in (left_eye, right_eye):
        pygame.draw.circle(surface, (121, 126, 130), (ex, ey), 9)
        pygame.draw.circle(surface, COLORS["robot_eye"], (ex, ey), 7)
        pygame.draw.circle(surface, COLORS["robot_pupil"], (ex + 1, ey + 1), 3)
        pygame.draw.circle(surface, COLORS["robot_glint"], (ex - 2, ey - 2), 2)

    # Small rescue light
    pygame.draw.circle(surface, (255, 223, 82), (x + w - 12, y + 18), 3)
    pygame.draw.line(surface, (255, 223, 82), (x + w - 12, y + 12), (x + w - 12, y + 8), 1)

def _draw_survivor(pygame, surface, loc, completed=False):
    x, y, w, h = _rect_for(loc)
    cx, cy = x + w // 2, y + h // 2

    if completed:
        # Once rescued, replace the person marker with a clear success badge.
        # This avoids the impression that a survivor is still waiting.
        pygame.draw.circle(surface, (230, 255, 236), (cx, cy), 18)
        pygame.draw.circle(surface, COLORS["survivor_done"], (cx, cy), 17, 3)
        pygame.draw.line(surface, COLORS["survivor_done"], (cx - 8, cy), (cx - 2, cy + 7), 4)
        pygame.draw.line(surface, COLORS["survivor_done"], (cx - 2, cy + 7), (cx + 10, cy - 8), 4)
        return

    color = COLORS["survivor"]
    pygame.draw.circle(surface, color, (cx, y + 16), 7)
    pygame.draw.rect(surface, COLORS["survivor_clothes"], (cx - 8, y + 23, 16, 15), border_radius=6)
    pygame.draw.line(surface, COLORS["survivor_clothes"], (cx - 8, y + 28), (cx - 15, y + 25), 3)
    pygame.draw.line(surface, COLORS["survivor_clothes"], (cx + 8, y + 28), (cx + 15, y + 25), 3)
    cross = (238, 246, 245)
    pygame.draw.circle(surface, (225, 74, 84), (x + w - 12, y + 13), 7)
    pygame.draw.line(surface, cross, (x+w-12, y+9), (x+w-12, y+17), 2)
    pygame.draw.line(surface, cross, (x+w-16, y+13), (x+w-8, y+13), 2)

def _draw_beacon(pygame, surface, loc, label, completed=False, small_font=None):
    x, y, w, h = _rect_for(loc)
    cx, cy = x + w//2, y + h//2
    color = COLORS["beacon_done"] if completed else COLORS["beacon"]
    pygame.draw.circle(surface, (245, 240, 255), (cx, cy), 20)
    pygame.draw.circle(surface, color, (cx, cy), 19, 3)
    pygame.draw.circle(surface, color, (cx, cy), 12, 2)
    pygame.draw.circle(surface, color, (cx, cy), 6)
    # little antenna/radio base
    pygame.draw.line(surface, color, (cx, cy + 6), (cx, cy + 17), 3)
    pygame.draw.arc(surface, color, (cx-23, cy-23, 46, 46), math.radians(215), math.radians(325), 2)
    if completed:
        pygame.draw.line(surface, (33, 119, 168), (cx - 7, cy), (cx - 2, cy + 6), 3)
        pygame.draw.line(surface, (33, 119, 168), (cx - 2, cy + 6), (cx + 9, cy - 8), 3)
    if small_font is not None:
        text = small_font.render(label, True, (62, 47, 110))
        surface.blit(text, (x + 5, y + 4))

def _draw_battery_bar(pygame, surface, mission, battery, x, y, width=165, height=16):
    pygame.draw.rect(surface, COLORS["battery_bg"], (x, y, width, height), border_radius=6)
    ratio = 0 if mission.initial_battery <= 0 else max(0, min(1, battery / mission.initial_battery))
    fill_width = int(width * ratio)
    if ratio < 0.25:
        color = COLORS["battery_low"]
    elif ratio < 0.55:
        color = COLORS["battery_mid"]
    else:
        color = COLORS["battery"]
    pygame.draw.rect(surface, color, (x, y, fill_width, height), border_radius=6)
    pygame.draw.rect(surface, COLORS["muted"], (x, y, width, height), 1, border_radius=6)
    # battery nub
    pygame.draw.rect(surface, COLORS["muted"], (x + width + 2, y + 4, 5, height - 8), border_radius=2)

def _draw_legend_chip(pygame, surface, font, x, y, label, color, kind="square"):
    icon_y = y + 2
    if kind == "circle":
        pygame.draw.circle(surface, color, (x + 7, icon_y + 7), 7)
    elif kind == "ring":
        pygame.draw.circle(surface, color, (x + 7, icon_y + 7), 8, 2)
        pygame.draw.circle(surface, color, (x + 7, icon_y + 7), 3)
    elif kind == "arrow":
        pygame.draw.line(surface, color, (x, icon_y + 7), (x + 15, icon_y + 7), 3)
        pygame.draw.polygon(surface, color, [(x + 16, icon_y + 7), (x + 9, icon_y + 2), (x + 9, icon_y + 12)])
    elif kind == "charge":
        pygame.draw.rect(surface, color, (x, icon_y, 15, 15), border_radius=3)
        pygame.draw.polygon(surface, (255, 245, 135), [(x + 8, icon_y + 2), (x + 4, icon_y + 8), (x + 8, icon_y + 8), (x + 6, icon_y + 13), (x + 12, icon_y + 6), (x + 8, icon_y + 6)])
    else:
        pygame.draw.rect(surface, color, (x, icon_y, 15, 15), border_radius=3)
        pygame.draw.rect(surface, COLORS["grid"], (x, icon_y, 15, 15), 1, border_radius=3)
    text = font.render(label, True, COLORS["muted"])
    surface.blit(text, (x + 20, y))
    return x + 26 + text.get_width()

def _draw_color_legend(pygame, surface, font, x, y):
    title = font.render("Key:", True, COLORS["text"])
    surface.blit(title, (x, y))
    row_x = x + title.get_width() + 12
    row_x = _draw_legend_chip(pygame, surface, font, row_x, y, "bot", COLORS["robot_yellow"], "circle")
    row_x = _draw_legend_chip(pygame, surface, font, row_x, y, "route", COLORS["route"], "arrow")
    row_x = _draw_legend_chip(pygame, surface, font, row_x, y, "survivor", COLORS["survivor"], "circle")
    _draw_legend_chip(pygame, surface, font, row_x, y, "beacon", COLORS["beacon"], "ring")

    row_x = x
    row_y = y + 22
    row_x = _draw_legend_chip(pygame, surface, font, row_x, row_y, "sand 2", COLORS["sand"])
    row_x = _draw_legend_chip(pygame, surface, font, row_x, row_y, "rubble 3", COLORS["rubble_light"])
    row_x = _draw_legend_chip(pygame, surface, font, row_x, row_y, "water 4", COLORS["water"])
    row_x = _draw_legend_chip(pygame, surface, font, row_x, row_y, "fire 8", COLORS["fire"])
    _draw_legend_chip(pygame, surface, font, row_x, row_y, "charge", COLORS["charge"], "charge")

def _draw_scene(pygame, screen, mission, robot_loc, battery, total_cost,
                path_locs=(), rescued=None, inspected=None, message="", controls=False):
    rescued = rescued or set()
    inspected = inspected or set()
    font = pygame.font.SysFont("arial", 18, bold=True)
    small = pygame.font.SysFont("arial", 14)
    tiny = pygame.font.SysFont("arial", 12, bold=True)

    screen.fill(COLORS["bg"])

    # Mission floor
    for r, row in enumerate(mission.grid):
        for c, ch in enumerate(row):
            loc = type(robot_loc)(r, c)
            rect = pygame.Rect(_rect_for(loc))
            pygame.draw.rect(screen, _tile_color(ch, r, c), rect, border_radius=4)
            _draw_tile_details(pygame, screen, ch, rect)
            pygame.draw.rect(screen, COLORS["grid"], rect, 1, border_radius=4)

    # Route overlay
    route = list(path_locs)
    if route:
        for i, loc in enumerate(route):
            if loc == robot_loc:
                continue
            prev_loc = route[i - 1] if i > 0 else None
            next_loc = route[i + 1] if i + 1 < len(route) else None
            _draw_route_marker(pygame, screen, loc, prev_loc, next_loc)

    # Mission targets
    for label, loc in mission.survivors.items():
        _draw_survivor(pygame, screen, loc, completed=(label in rescued))
    for label, loc in mission.beacons.items():
        _draw_beacon(pygame, screen, loc, label, completed=(label in inspected), small_font=tiny)

    # Robot
    _draw_robot(pygame, screen, robot_loc)

    # HUD panel. Keep the text on several short lines so it does not
    # disappear off the right side of the window on smaller maps.
    hud_y = PADDING + mission.height * TILE + 12
    window_w, _ = _window_size(mission)
    panel_w = window_w - PADDING * 2
    pygame.draw.rect(
        screen, COLORS["panel"],
        (PADDING, hud_y - 6, panel_w, HUD - 18),
        border_radius=10,
    )
    mission_score = 100 * len(rescued) + 25 * len(inspected) - total_cost + battery

    title_font = pygame.font.SysFont("arial", 17, bold=True)
    info_font = pygame.font.SysFont("arial", 14)
    legend_font = pygame.font.SysFont("arial", 12)

    left_x = PADDING + 14
    top_y = hud_y + 4

    line1 = (
        f"COST: {total_cost}    "
        f"BATTERY: {battery}/{mission.initial_battery}    "
        f"SCORE: {mission_score}"
    )
    screen.blit(title_font.render(line1, True, COLORS["text"]), (left_x, top_y))

    line2 = (
        f"MAP: {mission.name}    "
        f"Survivors: {len(rescued)}/{len(mission.survivors)}    "
        f"Beacons: {len(inspected)}/{len(mission.beacons)}"
    )
    screen.blit(info_font.render(line2, True, COLORS["text"]), (left_x, top_y + 28))

    _draw_battery_bar(pygame, screen, mission, battery, left_x, top_y + 56, width=210, height=18)

    _draw_color_legend(pygame, screen, legend_font, left_x + 240, top_y + 55)

    bottom_line = ""
    if controls:
        bottom_line = "Manual mode: Arrow keys/WASD move   R reset   Esc/Q quit"
    if message:
        bottom_line = (bottom_line + "   |   " if bottom_line else "") + message
    if bottom_line:
        screen.blit(info_font.render(bottom_line, True, COLORS["text"]), (left_x, top_y + 124))

    pygame.display.flip()

def animate(mission, frames, plan=None, delay=0.08, title="4102 Rescue Robot Search"):
    """Animate a planned route and update mission progress as the robot moves.

    The planner returns only a list of actions, so the simulator frames contain
    location, battery, and cost. The display reconstructs which survivors and
    beacons have been reached from the locations visited so far.
    """
    pygame = _require_pygame()
    pygame.init()
    screen = pygame.display.set_mode(_window_size(mission))
    pygame.display.set_caption(title)

    path_locs = [f[0] for f in frames] if frames else []
    visited = []
    rescued = set()
    inspected = set()

    def update_progress(loc):
        visited.append(loc)
        for label, target in mission.survivors.items():
            if target == loc:
                rescued.add(label)
        for label, target in mission.beacons.items():
            if target == loc:
                inspected.add(label)

    running = True
    for loc, battery, total_cost in frames:
        update_progress(loc)
        start = time.time()
        while time.time() - start < delay:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
            if not running:
                pygame.quit()
                return
            time.sleep(0.005)
        message = "Mission complete" if loc == frames[-1][0] else "Executing planned route"
        _draw_scene(
            pygame, screen, mission, loc, battery, total_cost,
            path_locs=path_locs, rescued=rescued, inspected=inspected,
            message=message
        )

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        time.sleep(0.02)
    pygame.quit()

def manual_play(mission, title="4102 Rescue Robot Manual Mode"):
    """Keyboard-controlled preview mode.

    This is for exploration and instructor/student visualization. It does not
    call planner.py, objectives.py, or heuristics.py, so it works even before
    students complete the programming questions.
    """
    pygame = _require_pygame()
    from rescue_core.actions import ACTIONS

    pygame.init()
    screen = pygame.display.set_mode(_window_size(mission))
    pygame.display.set_caption(title)
    clock = pygame.time.Clock()

    key_to_action = {
        pygame.K_UP: "N", pygame.K_w: "N",
        pygame.K_DOWN: "S", pygame.K_s: "S",
        pygame.K_LEFT: "W", pygame.K_a: "W",
        pygame.K_RIGHT: "E", pygame.K_d: "E",
    }

    loc = mission.start
    battery = mission.initial_battery
    total_cost = 0
    trail = [loc]
    rescued: set[str] = set()
    inspected: set[str] = set()
    message = "Manual mode"

    def reset():
        nonlocal loc, battery, total_cost, trail, rescued, inspected, message
        loc = mission.start
        battery = mission.initial_battery
        total_cost = 0
        trail = [loc]
        rescued = set()
        inspected = set()
        message = "Manual mode"

    def collect_at_current_location():
        nonlocal rescued, inspected
        for label, target in mission.survivors.items():
            if target == loc:
                rescued.add(label)
        for label, target in mission.beacons.items():
            if target == loc:
                inspected.add(label)

    collect_at_current_location()
    running = True
    while running:
        _draw_scene(
            pygame, screen, mission, loc, battery, total_cost,
            path_locs=trail, rescued=rescued, inspected=inspected,
            message=message, controls=True
        )

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.key == pygame.K_r:
                    reset()
                elif event.key in key_to_action:
                    action = key_to_action[event.key]
                    dr, dc = ACTIONS[action]
                    nxt = loc.moved(dr, dc)
                    if mission.is_blocked(nxt):
                        message = "Blocked route"
                        continue
                    step_cost = mission.movement_cost(nxt)
                    if battery - step_cost < 0:
                        message = "Battery depleted"
                        continue
                    loc = nxt
                    total_cost += step_cost
                    battery -= step_cost
                    if mission.is_charger(loc):
                        old_battery = battery
                        battery = mission.recharge_after_entering(loc, battery)
                        message = f"Charging station: +{battery - old_battery}"
                    else:
                        message = f"Moved {action}, cost {step_cost}"
                    trail.append(loc)
                    collect_at_current_location()

        clock.tick(30)
    pygame.quit()
