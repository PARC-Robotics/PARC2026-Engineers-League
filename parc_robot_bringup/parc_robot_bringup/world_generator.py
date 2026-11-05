"""Seeded generator for the randomised PARC arena.

The arena is a 7 m x 6 m walled rectangle. Four AprilTag signs (IDs 1, 4, 7, 9,
centre at 1.0 m height) are placed on the walls or on free-standing posts, and
everyday obstacles are scattered between them. Every world is reachable: the
visit pose 0.5 m in front of each tag, and the approach 1.0 m in front of it,
can be reached from the start with `ROBOT_CLEARANCE` metres of free space around
the robot centre.

    python3 -m parc_robot_bringup.world_generator --seed 7 --out /tmp/parc_worlds
"""

import argparse
import heapq
import json
import math
import os
import random
from dataclasses import dataclass, field

import numpy as np

XMIN, XMAX, YMIN, YMAX = -3.5, 3.5, -8.0, -2.0
START = (0.0, -7.0, math.pi / 2)
TAG_IDS = (1, 4, 7, 9)
TAG_HEIGHT = 1.0
TAG_SIZE = 0.15
VISIT_DISTANCE = 0.5
APPROACH_DISTANCE = 1.0

ROBOT_CLEARANCE = 0.40
WALL_CLEARANCE = 0.32
OBSTACLE_GAP = 0.15
GRID = 0.05

TAG_WALL_MARGIN = 1.0
FREE_TAG_MARGIN = 1.4
TAG_MIN_SEPARATION = 2.2
TAG_START_SEPARATION = 1.8
START_KEEP_OUT_RADIUS = 0.9
TAG_KEEP_OUT_DEPTH = 1.5
TAG_KEEP_OUT_HALF_WIDTH = 0.6

# Roll (quarter turns) of the tag face that makes the texture appear upright.
TAG_QUARTER_TURNS = 2

WALL_HEIGHT = 0.4
WALL_THICKNESS = 0.02


# --------------------------------------------------------------------- shapes
class Box:
    def __init__(self, cx, cy, hx, hy, yaw):
        self.cx, self.cy, self.hx, self.hy, self.yaw = cx, cy, hx, hy, yaw

    def distance(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx = (x - self.cx) * c + (y - self.cy) * s
        ly = -(x - self.cx) * s + (y - self.cy) * c
        dx, dy = np.abs(lx) - self.hx, np.abs(ly) - self.hy
        return np.hypot(np.maximum(dx, 0), np.maximum(dy, 0)) + np.minimum(
            np.maximum(dx, dy), 0
        )

    def extreme_points(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [
            (
                self.cx + sx * self.hx * c - sy * self.hy * s,
                self.cy + sx * self.hx * s + sy * self.hy * c,
            )
            for sx in (-1, 1)
            for sy in (-1, 1)
        ]


class Circle:
    def __init__(self, cx, cy, r):
        self.cx, self.cy, self.r = cx, cy, r

    def distance(self, x, y):
        return np.hypot(x - self.cx, y - self.cy) - self.r

    def extreme_points(self):
        return [
            (self.cx + dx * self.r, self.cy + dy * self.r)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))
        ]


def local_box(x, y, yaw, ox, oy, hx, hy):
    c, s = math.cos(yaw), math.sin(yaw)
    return Box(x + ox * c - oy * s, y + ox * s + oy * c, hx, hy, yaw)


# ------------------------------------------------------------------ SDF parts
def fmt_pose(x, y, z=0.0, yaw=0.0):
    return f"{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}"


def material(rgba):
    r, g, b, a = rgba
    return (
        f"<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>"
        "<specular>0.1 0.1 0.1 1</specular></material>"
    )


def geometry(kind, dims):
    if kind == "box":
        return f"<box><size>{dims[0]} {dims[1]} {dims[2]}</size></box>"
    if kind == "cylinder":
        return f"<cylinder><radius>{dims[0]}</radius><length>{dims[1]}</length></cylinder>"
    return f"<sphere><radius>{dims[0]}</radius></sphere>"


def part(index, kind, dims, x, y, z, rgba):
    geo = geometry(kind, dims)
    return (
        f"<collision name='c{index}'><pose>{x} {y} {z} 0 0 0</pose><geometry>{geo}</geometry></collision>"
        f"<visual name='v{index}'><pose>{x} {y} {z} 0 0 0</pose><geometry>{geo}</geometry>{material(rgba)}</visual>"
    )


def primitive_model(name, x, y, yaw, parts):
    body = "".join(part(i, *p) for i, p in enumerate(parts))
    return (
        f"<model name='{name}'><static>true</static><pose>{fmt_pose(x, y, 0.0, yaw)}</pose>"
        f"<link name='link'>{body}</link></model>"
    )


def include(name, uri, x, y, yaw, z=0.0):
    return (
        f"<include><name>{name}</name><uri>model://{uri}</uri>"
        f"<pose>{fmt_pose(x, y, z, yaw)}</pose></include>"
    )


# ------------------------------------------------------------------ obstacles
@dataclass
class Obstacle:
    kind: str
    x: float
    y: float
    yaw: float
    shapes: list
    sdf: str
    meta: dict = field(default_factory=dict)


CARTON = (0.74, 0.56, 0.35, 1)
GREY = (0.45, 0.47, 0.50, 1)
DARK = (0.18, 0.19, 0.21, 1)
TERRACOTTA = (0.72, 0.38, 0.26, 1)
GREEN = (0.20, 0.45, 0.22, 1)
WOOD = (0.55, 0.38, 0.22, 1)
NAVY = (0.12, 0.17, 0.38, 1)
SOFA = (0.50, 0.22, 0.20, 1)


def build_table_set(rng, name, x, y, yaw):
    parts = [include(f"{name}_table", "cafe_table", x, y, yaw)]
    shapes = [Box(x, y, 0.46, 0.46, yaw)]
    chair_count = rng.choice([1, 2, 2])
    base = rng.uniform(0, 2 * math.pi)
    visitor_chair = 0 if rng.random() < 0.5 else None
    for k in range(chair_count):
        angle = base + k * math.pi + rng.uniform(-0.4, 0.4)
        distance = rng.uniform(0.42, 0.62)
        cx, cy = x + distance * math.cos(angle), y + distance * math.sin(angle)
        facing = angle + math.pi + rng.uniform(-0.15, 0.15)
        parts.append(include(f"{name}_chair{k}", "WoodenChair", cx, cy, facing))
        shapes.append(local_box(cx, cy, facing, -0.02, 0.0, 0.22, 0.20))
        if k == visitor_chair:
            model = rng.choice(["FemaleVisitorSit", "MaleVisitorSit"])
            vyaw = facing + math.pi / 2
            parts.append(include(f"{name}_visitor", model, cx, cy, vyaw))
            shapes.append(local_box(cx, cy, vyaw, 0.0, -0.18, 0.22, 0.33))
    return Obstacle("table_set", x, y, yaw, shapes, "".join(parts))


def simple(kind, parts_fn, shapes_fn):
    def build(rng, name, x, y, yaw):
        return Obstacle(
            kind, x, y, yaw, shapes_fn(x, y, yaw), primitive_model(name, x, y, yaw, parts_fn(rng))
        )

    return build


# Free-standing catalogue. Parts are (kind, dims, x, y, z, rgba) in the model frame.
FREE_OBSTACLES = {
    "carton": simple(
        "carton",
        lambda r: [("box", (0.5, 0.4, 0.4), 0, 0, 0.2, CARTON)],
        lambda x, y, yaw: [Box(x, y, 0.25, 0.20, yaw)],
    ),
    "carton_stack": simple(
        "carton_stack",
        lambda r: [
            ("box", (0.6, 0.45, 0.4), 0, 0, 0.2, CARTON),
            ("box", (0.45, 0.35, 0.35), 0.03, 0.02, 0.575, CARTON),
        ],
        lambda x, y, yaw: [Box(x, y, 0.30, 0.225, yaw)],
    ),
    "trash_bin": simple(
        "trash_bin",
        lambda r: [("cylinder", (0.19, 0.65), 0, 0, 0.325, DARK)],
        lambda x, y, yaw: [Circle(x, y, 0.19)],
    ),
    "plant": simple(
        "plant",
        lambda r: [
            ("cylinder", (0.20, 0.40), 0, 0, 0.20, TERRACOTTA),
            ("sphere", (0.33,), 0, 0, 0.75, GREEN),
        ],
        lambda x, y, yaw: [Circle(x, y, 0.33)],
    ),
    "stool": simple(
        "stool",
        lambda r: [("cylinder", (0.20, 0.45), 0, 0, 0.225, WOOD)],
        lambda x, y, yaw: [Circle(x, y, 0.20)],
    ),
    "suitcase": simple(
        "suitcase",
        lambda r: [("box", (0.45, 0.25, 0.70), 0, 0, 0.35, NAVY)],
        lambda x, y, yaw: [Box(x, y, 0.225, 0.125, yaw)],
    ),
}

# Wall-hugging catalogue. The front faces +x in the model frame; `depth` is the
# extent along x and the model origin is its centre unless noted.
WALL_OBSTACLES = {
    "sofa": (
        0.80,
        simple(
            "sofa",
            lambda r: [
                ("box", (0.8, 1.5, 0.45), 0, 0, 0.225, SOFA),
                ("box", (0.18, 1.5, 0.45), -0.31, 0, 0.675, SOFA),
            ],
            lambda x, y, yaw: [local_box(x, y, yaw, 0, 0, 0.40, 0.75)],
        ),
    ),
    "bookshelf": (
        0.30,
        simple(
            "bookshelf",
            lambda r: [("box", (0.30, 0.9, 1.6), 0, 0, 0.8, WOOD)],
            lambda x, y, yaw: [local_box(x, y, yaw, 0, 0, 0.15, 0.45)],
        ),
    ),
    "cabinet": (
        0.45,
        simple(
            "cabinet",
            lambda r: [("box", (0.45, 0.6, 1.2), 0, 0, 0.6, GREY)],
            lambda x, y, yaw: [local_box(x, y, yaw, 0, 0, 0.225, 0.30)],
        ),
    ),
    "vending_machine": (
        0.61,
        lambda rng, name, x, y, yaw: Obstacle(
            "vending_machine",
            x,
            y,
            yaw,
            [local_box(x, y, yaw, 0.007, 0.0, 0.31, 0.46)],
            include(name, "VendingMachine", x, y, yaw),
        ),
    ),
}

# Wall name -> angle of the inward-pointing normal
WALLS = {
    "west": 0.0,
    "east": math.pi,
    "south": math.pi / 2,
    "north": -math.pi / 2,
}


def wall_point(wall, t, inset):
    """Point `inset` metres inside the arena from a wall, `t` metres along it."""
    if wall == "west":
        return XMIN + inset, YMIN + t
    if wall == "east":
        return XMAX - inset, YMIN + t
    if wall == "south":
        return XMIN + t, YMIN + inset
    return XMIN + t, YMAX - inset


def wall_length(wall):
    return (YMAX - YMIN) if wall in ("west", "east") else (XMAX - XMIN)


# ------------------------------------------------------------------------ tags
@dataclass
class Tag:
    tag_id: int
    x: float
    y: float
    facing: float
    mount: str

    def visit_pose(self, distance=VISIT_DISTANCE):
        return (
            self.x + distance * math.cos(self.facing),
            self.y + distance * math.sin(self.facing),
            self.facing + math.pi,
        )

    def sign_shape(self):
        return Box(self.x, self.y, 0.03, 0.13, self.facing)

    def keep_out(self):
        return Box(
            self.x + (TAG_KEEP_OUT_DEPTH - 0.15) / 2 * math.cos(self.facing),
            self.y + (TAG_KEEP_OUT_DEPTH - 0.15) / 2 * math.sin(self.facing),
            (TAG_KEEP_OUT_DEPTH + 0.15) / 2,
            TAG_KEEP_OUT_HALF_WIDTH,
            self.facing,
        )


def tag_sign_sdf(tag, quarter_turns):
    """Sign with a post, a white panel and the tag texture on the panel's +x face."""
    post = part(0, "box", (0.03, 0.03, 1.15), -0.025, 0, 0.575, (0.76, 0.60, 0.42, 1))
    panel = part(1, "box", (0.012, 0.25, 0.25), 0, 0, TAG_HEIGHT, (0.92, 0.92, 0.92, 1))
    texture = (
        f"model://Apriltag36_11_{tag.tag_id:05d}/materials/textures/"
        f"tag36_11_{tag.tag_id:05d}.png"
    )
    face = (
        f"<visual name='tag'><pose>0.0066 0 {TAG_HEIGHT} {quarter_turns * math.pi / 2:.5f} 0 0</pose>"
        f"<geometry><box><size>0.001 {TAG_SIZE} {TAG_SIZE}</size></box></geometry>"
        "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
        f"<specular>0.5 0.5 0.5 1</specular><pbr><metal><albedo_map>{texture}</albedo_map>"
        "<roughness>0.5</roughness><metalness>0.0</metalness></metal></pbr></material></visual>"
    )
    return (
        f"<model name='tag_sign_{tag.tag_id}'><static>true</static>"
        f"<pose>{fmt_pose(tag.x, tag.y, 0.0, tag.facing)}</pose>"
        f"<link name='link'>{post}{panel}{face}</link></model>"
    )


# ------------------------------------------------------------------ the world
class Grid:
    def __init__(self):
        self.xs = np.arange(XMIN + GRID / 2, XMAX, GRID)
        self.ys = np.arange(YMIN + GRID / 2, YMAX, GRID)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)

    def mask(self, shapes, inflate=0.0):
        mask = np.zeros(self.X.shape, dtype=bool)
        for shape in shapes:
            mask |= shape.distance(self.X, self.Y) <= inflate
        return mask

    def cell(self, x, y):
        return (
            int(np.clip((y - YMIN) / GRID, 0, len(self.ys) - 1)),
            int(np.clip((x - XMIN) / GRID, 0, len(self.xs) - 1)),
        )

    def wall_blocked(self):
        edge = np.minimum.reduce(
            [self.X - XMIN, XMAX - self.X, self.Y - YMIN, YMAX - self.Y]
        )
        return edge <= WALL_CLEARANCE

    def distances_from(self, free, source):
        """Dijkstra over free cells (8-connected); returns metres, inf if unreachable."""
        dist = np.full(free.shape, np.inf)
        if not free[source]:
            return dist
        dist[source] = 0.0
        heap = [(0.0, source)]
        steps = [(dy, dx, math.hypot(dy, dx) * GRID) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
        rows, cols = free.shape
        while heap:
            d, (r, c) = heapq.heappop(heap)
            if d > dist[r, c]:
                continue
            for dr, dc, w in steps:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and free[nr, nc] and d + w < dist[nr, nc]:
                    dist[nr, nc] = d + w
                    heapq.heappush(heap, (d + w, (nr, nc)))
        return dist


@dataclass
class World:
    seed: int
    tags: list
    obstacles: list
    detour: float
    sdf: str

    def truth(self):
        return {
            "seed": self.seed,
            "arena": {"xmin": XMIN, "xmax": XMAX, "ymin": YMIN, "ymax": YMAX},
            "start": {"x": START[0], "y": START[1], "yaw": START[2]},
            "mean_detour_ratio": round(self.detour, 3),
            "tags": [
                {
                    "id": t.tag_id,
                    "x": round(t.x, 3),
                    "y": round(t.y, 3),
                    "z": TAG_HEIGHT,
                    "facing_yaw": round(t.facing, 4),
                    "mount": t.mount,
                    "visit_pose": [round(v, 3) for v in t.visit_pose()],
                }
                for t in self.tags
            ],
            "obstacles": [
                {"kind": o.kind, "x": round(o.x, 3), "y": round(o.y, 3), "yaw": round(o.yaw, 3)}
                for o in self.obstacles
            ],
        }


class WorldGenerator:
    def __init__(self, seed, obstacle_sets=3, wall_items=3, free_items=6, min_detour=1.2):
        self.seed = seed
        self.obstacle_sets = obstacle_sets
        self.wall_items = wall_items
        self.free_items = free_items
        self.min_detour = min_detour
        self.grid = Grid()

    def generate(self, max_attempts=400):
        rng = random.Random(self.seed)
        tags = self.place_tags(rng)
        fixed = [t.sign_shape() for t in tags]
        keep_out = self.grid.mask([Circle(START[0], START[1], START_KEEP_OUT_RADIUS)]) | self.grid.mask(
            [t.keep_out() for t in tags]
        )

        best = None
        for _ in range(max_attempts):
            obstacles = self.place_obstacles(rng, fixed, keep_out)
            if obstacles is None:
                continue
            detour = self.evaluate(tags, fixed, obstacles)
            if detour is None:
                continue
            if best is None or detour > best[0]:
                best = (detour, obstacles)
            if detour >= self.min_detour:
                break
        if best is None:
            raise RuntimeError(f"No reachable layout found for seed {self.seed}")
        detour, obstacles = best
        return World(self.seed, tags, obstacles, detour, self.compose(tags, obstacles))

    # -- tags
    def place_tags(self, rng):
        for _ in range(2000):
            free_count = rng.choice([0, 1, 1, 2])
            ids = list(TAG_IDS)
            rng.shuffle(ids)
            tags = [
                self.sample_tag(rng, tag_id, "free" if i < free_count else "wall")
                for i, tag_id in enumerate(ids)
            ]
            if self.tags_valid(tags):
                return tags
        raise RuntimeError("Could not place tags")

    @staticmethod
    def sample_tag(rng, tag_id, mount):
        if mount == "free":
            x = rng.uniform(XMIN + FREE_TAG_MARGIN, XMAX - FREE_TAG_MARGIN)
            y = rng.uniform(YMIN + FREE_TAG_MARGIN, YMAX - FREE_TAG_MARGIN)
            return Tag(tag_id, x, y, rng.uniform(-math.pi, math.pi), "free")
        wall = rng.choice(list(WALLS))
        t = rng.uniform(TAG_WALL_MARGIN, wall_length(wall) - TAG_WALL_MARGIN)
        x, y = wall_point(wall, t, 0.03)
        return Tag(tag_id, x, y, WALLS[wall], wall)

    @staticmethod
    def tags_valid(tags):
        for i, a in enumerate(tags):
            if math.hypot(a.x - START[0], a.y - START[1]) < TAG_START_SEPARATION:
                return False
            for b in tags[i + 1 :]:
                if math.hypot(a.x - b.x, a.y - b.y) < TAG_MIN_SEPARATION:
                    return False
        return True

    # -- obstacles
    def place_obstacles(self, rng, fixed, keep_out):
        placed = []
        occupied = self.grid.mask(fixed, OBSTACLE_GAP)
        counter = 0

        def try_place(make):
            nonlocal occupied, counter
            for _ in range(60):
                obstacle = make(f"obs{counter}")
                if not all(self.inside(s) for s in obstacle.shapes):
                    continue
                footprint = self.grid.mask(obstacle.shapes)
                if (footprint & (occupied | keep_out)).any():
                    continue
                occupied |= self.grid.mask(obstacle.shapes, OBSTACLE_GAP)
                placed.append(obstacle)
                counter += 1
                return True
            return False

        def random_free(builder):
            def make(name):
                return builder(
                    rng,
                    name,
                    rng.uniform(XMIN + 0.4, XMAX - 0.4),
                    rng.uniform(YMIN + 0.4, YMAX - 0.4),
                    rng.uniform(-math.pi, math.pi),
                )

            return make

        def random_wall(depth, builder):
            def make(name):
                wall = rng.choice(list(WALLS))
                t = rng.uniform(0.5, wall_length(wall) - 0.5)
                x, y = wall_point(wall, t, depth / 2 + 0.03)
                return builder(rng, name, x, y, WALLS[wall])

            return make

        for _ in range(self.obstacle_sets):
            if not try_place(random_free(build_table_set)):
                return None
        for kind in rng.sample(list(WALL_OBSTACLES), min(self.wall_items, len(WALL_OBSTACLES))):
            depth, builder = WALL_OBSTACLES[kind]
            if not try_place(random_wall(depth, builder)):
                return None
        for _ in range(self.free_items):
            builder = FREE_OBSTACLES[rng.choice(list(FREE_OBSTACLES))]
            try_place(random_free(builder))
        return placed

    @staticmethod
    def inside(shape):
        return all(
            XMIN + 0.03 <= x <= XMAX - 0.03 and YMIN + 0.03 <= y <= YMAX - 0.03
            for x, y in shape.extreme_points()
        )

    # -- reachability and difficulty
    def evaluate(self, tags, fixed, obstacles):
        shapes = fixed + [s for o in obstacles for s in o.shapes]
        free = ~(self.grid.mask(shapes, ROBOT_CLEARANCE) | self.grid.wall_blocked())

        points = [START[:2]] + [t.visit_pose(VISIT_DISTANCE)[:2] for t in tags]
        approaches = [t.visit_pose(APPROACH_DISTANCE)[:2] for t in tags]
        cells = [self.grid.cell(*p) for p in points]
        approach_cells = [self.grid.cell(*p) for p in approaches]
        if not all(free[c] for c in cells + approach_cells):
            return None

        ratios = []
        for i, source in enumerate(cells):
            dist = self.grid.distances_from(free, source)
            if i == 0 and not all(np.isfinite(dist[c]) for c in approach_cells):
                return None
            for j in range(i + 1, len(cells)):
                path = dist[cells[j]]
                if not math.isfinite(path):
                    return None
                straight = math.hypot(
                    points[i][0] - points[j][0], points[i][1] - points[j][1]
                )
                ratios.append(path / max(straight, 1e-6))
        return float(np.mean(ratios))

    # -- SDF
    def compose(self, tags, obstacles):
        parts = [WORLD_HEADER, self.walls_sdf()]
        parts.extend(tag_sign_sdf(t, TAG_QUARTER_TURNS) for t in tags)
        parts.extend(o.sdf for o in obstacles)
        parts.append(WORLD_FOOTER)
        return "\n".join(parts)

    @staticmethod
    def walls_sdf():
        sx, sy = XMAX - XMIN, YMAX - YMIN
        cx, cy = (XMAX + XMIN) / 2, (YMAX + YMIN) / 2
        walls = [
            ("wall_south", cx, YMIN, sx, WALL_THICKNESS),
            ("wall_north", cx, YMAX, sx, WALL_THICKNESS),
            ("wall_west", XMIN, cy, WALL_THICKNESS, sy),
            ("wall_east", XMAX, cy, WALL_THICKNESS, sy),
        ]
        return "".join(
            primitive_model(name, x, y, 0.0, [("box", (w, h, WALL_HEIGHT), 0, 0, WALL_HEIGHT / 2, (0.76, 0.60, 0.42, 1))])
            for name, x, y, w, h in walls
        )


WORLD_HEADER = """<?xml version="1.0"?>
<sdf version='1.10'>
  <world name='parc_random'>
    <physics name="10ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name='visual'>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <light name='sun' type='directional'>
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>"""

WORLD_FOOTER = """  </world>
</sdf>
"""


def generate(seed, **options):
    return WorldGenerator(seed, **options).generate()


def write_world(world, directory):
    os.makedirs(directory, exist_ok=True)
    sdf_path = os.path.join(directory, f"parc_random_seed{world.seed}.sdf")
    json_path = os.path.join(directory, f"parc_random_seed{world.seed}.json")
    with open(sdf_path, "w") as f:
        f.write(world.sdf)
    with open(json_path, "w") as f:
        json.dump(world.truth(), f, indent=2)
    return sdf_path, json_path


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out", default="/tmp/parc_worlds")
    parser.add_argument("--min-detour", type=float, default=1.2)
    args = parser.parse_args()
    world = generate(args.seed, min_detour=args.min_detour)
    sdf_path, json_path = write_world(world, args.out)
    print(f"world: {sdf_path}\nground truth: {json_path}\nmean detour ratio: {world.detour:.2f}")
    for tag in world.tags:
        print(f"tag {tag.tag_id}: {tag.mount:5s} at ({tag.x:6.2f}, {tag.y:6.2f}) facing {tag.facing:5.2f}")


if __name__ == "__main__":
    main()
