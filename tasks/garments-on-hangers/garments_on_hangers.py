"""On the station table: a clothes rail (fixed) runs across the back of the work area with two open-ended hangers
hanging from it (fixed): a wooden trouser hanger on the left and a pink padded hanger, for delicates, on the right.
Each hanger's bar runs left-right and is held only at its outer end, so a folded garment can be laid over it from
above. In front of the rail stands a low white shelf (fixed). Three folded garments lie on the table (loose): navy
trousers, an ivory silk blouse and a grey wool sweater.

  seen from above (y in cm)    trouser hanger bar y +5..+15     shelf y -6..+6 (x 16..28)    padded bar y -5..-15
                               (both bars at x 30, 10 cm up)
  table, x 17:                 sweater y +20     trousers y +10                               blouse y -10

The trousers go over the trouser hanger's bar and the blouse over the padded hanger's bar, each folded across the
bar with its middle on it; the sweater lies flat on the shelf (knits stretch out of shape on a hanger).

Run it:  inspect-robots run --embodiment yam_arena -E task=garments_on_hangers.py -E object_jitter=0 \\
           --scorer task_score --instruction "Put the clean laundry away: ..." ...
"""

import sys

import numpy as np

from yam_arena import YAM_URDF
from yam_arena.analysis import link_hulls
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_JOINTS, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SPAWN_GAP = 0.001

# -- fixed: the rail, two hangers and the shelf ----------------------------------------------------------------

HANGER_X = 0.30
BAR_TOP_H = 0.10  # each hanger bar's top face this far above the table
RAIL_H = 0.22
RAIL_GREY, STEM_GREY = (0.62, 0.63, 0.66), (0.55, 0.56, 0.58)

# hanger: (bar centre y, bar size x (front-back), y (left-right), z; colour; the end its stem holds: +1 left, -1 right)
HANGERS = {
    "trouser_hanger": (0.10, (0.025, 0.10, 0.012), (0.45, 0.30, 0.16), +1),  # flat varnished wooden bar
    "padded_hanger": (-0.10, (0.030, 0.10, 0.022), (0.93, 0.62, 0.72), -1),  # thick satin-padded bar
}
STEM = 0.006  # the hook's neck: a thin square rod from the bar's outer end up to the rail


def _hanger(name: str) -> list[Box]:
    cy, size, colour, side = HANGERS[name]
    bar_top = TOP + BAR_TOP_H
    stem_y = cy + side * (size[1] / 2 + STEM / 2)
    stem_bottom = bar_top - size[2]
    stem_len = TOP + RAIL_H - stem_bottom
    return [
        Box(f"{name}_bar", pos=(HANGER_X, cy, bar_top - size[2] / 2), size=size, color=colour),
        Box(f"{name}_stem", pos=(HANGER_X, stem_y, stem_bottom + stem_len / 2), size=(STEM, STEM, stem_len),
            color=STEM_GREY),
    ]


RAIL = [
    Box("rail", pos=(HANGER_X, 0.0, TOP + RAIL_H + 0.0075), size=(0.015, 0.48, 0.015), color=RAIL_GREY),
    Box("rail_post_left", pos=(HANGER_X, 0.235, TOP + RAIL_H / 2), size=(0.015, 0.015, RAIL_H), color=RAIL_GREY),
    Box("rail_post_right", pos=(HANGER_X, -0.235, TOP + RAIL_H / 2), size=(0.015, 0.015, RAIL_H), color=RAIL_GREY),
]

SHELF_CENTRE = (0.22, 0.0)
SHELF_SIZE = (0.12, 0.12, 0.04)  # x, y, height
SHELF_TOP = TOP + SHELF_SIZE[2]
SHELF = Box("shelf", pos=(*SHELF_CENTRE, TOP + SHELF_SIZE[2] / 2), size=SHELF_SIZE, color=(0.94, 0.94, 0.92))

# -- loose: three folded garments on the table, x = 17 cm, long side along x ------------------------------------

ROW_X = 0.17
# garment: (size x, y, z; mass; colour; start y; destination)
GARMENTS = {
    "trousers": ((0.14, 0.045, 0.020), 0.10, (0.12, 0.16, 0.36), 0.10, "trouser_hanger"),
    "silk_blouse": ((0.12, 0.045, 0.016), 0.04, (0.96, 0.93, 0.84), -0.10, "padded_hanger"),
    "wool_sweater": ((0.08, 0.050, 0.025), 0.12, (0.52, 0.53, 0.55), 0.20, "shelf"),
}
ITEMS = {
    n: Box(n, pos=(ROW_X, y, TOP + size[2] / 2 + SPAWN_GAP), size=size, mass=m, color=c)
    for n, (size, m, c, y, _) in GARMENTS.items()
}
DEST = {n: g[4] for n, g in GARMENTS.items()}

OBJECTS = [*RAIL, *(b for h in HANGERS for b in _hanger(h)), SHELF, *ITEMS.values()]
SPECS = dict(ITEMS)

# Scoring thresholds (m, rad).
DROPPED = 0.05
REST_TOL = 0.005  # a garment's underside within this of the bar's or shelf's top face
LEVEL_TOL = np.radians(10.0)  # folded over a bar or lying on the shelf: tilted no more than this
ACROSS_TOL = np.radians(15.0)  # on a bar: the garment's long axis within this of x, across the bar
RELEASED = 0.03
PARTIAL_CAP = 0.9
SUCCESS_HOLD_S = 1.0


# -- geometry ------------------------------------------------------------------------------------------------


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, dtype=float))


def extent(state, spec) -> tuple[np.ndarray, np.ndarray]:
    half = np.abs(_rot(state.quat_xyzw)) @ (np.asarray(spec.size) / 2)
    centre = np.asarray(state.pos, dtype=float)
    return centre - half, centre + half


def fell(state, spec) -> bool:
    return bool(extent(state, spec)[0][2] < TOP - DROPPED)


def tilt(state) -> float:
    return float(np.arccos(np.clip(abs(_rot(state.quat_xyzw)[2, 2]), -1.0, 1.0)))


def across(state) -> bool:
    axis = _rot(state.quat_xyzw)[:, 0]
    return bool(np.arctan2(abs(axis[1]), abs(axis[0])) <= ACROSS_TOL)


def on_bar(state, spec, hanger: str) -> bool:
    """Folded over the hanger's bar: lying across it (long axis within 15 degrees of x, tilt under 10 degrees),
    its underside within 5 mm of the bar's top face, its centre over the bar (within the bar's front-back width)
    and the whole of its width within the bar's length."""
    cy, size, _, _ = HANGERS[hanger]
    lo, hi = extent(state, spec)
    return bool(across(state) and tilt(state) <= LEVEL_TOL
                and abs(lo[2] - (TOP + BAR_TOP_H)) <= REST_TOL
                and abs(state.pos[0] - HANGER_X) <= size[0] / 2
                and lo[1] >= cy - size[1] / 2 and hi[1] <= cy + size[1] / 2)


def on_shelf(state, spec) -> bool:
    """Lying flat on the shelf: tilt under 10 degrees, its underside within 5 mm of the shelf top, and wholly over
    the shelf's top face."""
    lo, hi = extent(state, spec)
    sx, sy = SHELF_CENTRE
    hx, hy = SHELF_SIZE[0] / 2, SHELF_SIZE[1] / 2
    return bool(tilt(state) <= LEVEL_TOL and abs(lo[2] - SHELF_TOP) <= REST_TOL
                and lo[0] >= sx - hx and hi[0] <= sx + hx and lo[1] >= sy - hy and hi[1] <= sy + hy)


def where(state, spec) -> str | None:
    if on_shelf(state, spec):
        return "shelf"
    for h in HANGERS:
        if on_bar(state, spec, h):
            return h
    return None


def tally(objects) -> dict:
    at = {n: where(objects[n], s) for n, s in SPECS.items()}
    return {
        "fell": [n for n, s in SPECS.items() if fell(objects[n], s)],
        "correct": [n for n, d in at.items() if d == DEST[n]],
        "wrong": [n for n, d in at.items() if d is not None and d != DEST[n]],
        "where": at,
    }


# -- the robot (release check) ------------------------------------------------------------------------------------

ROBOTS = ("left_robot", "right_robot")


def _fingertips_in_link() -> dict[str, np.ndarray]:
    poses = forward_kinematics(YAM_URDF, {})
    to_gripper = np.linalg.inv(poses["gripper"])
    tips = {}
    for link in ("tip_left", "tip_right"):
        verts = np.vstack([hull.vertices for hull in link_hulls()[link]])
        in_gripper = (to_gripper @ poses[link] @ np.c_[verts, np.ones(len(verts))].T).T[:, :3]
        far = in_gripper[:, 2] >= in_gripper[:, 2].max() - 0.001
        tips[link] = verts[far].mean(axis=0)
    return tips


FINGERTIPS = _fingertips_in_link()


def _embodiment():
    frame = sys._getframe(1)
    while frame is not None:
        owner = frame.f_locals.get("self")
        if owner is not None and hasattr(owner, "_task_step") and hasattr(owner, "_t"):
            return owner
        frame = frame.f_back
    raise RuntimeError("garments_on_hangers checks release only inside a yam_arena embodiment's step "
                       "(inspect-robots run --embodiment yam_arena)")


def fingertip_positions(embodiment) -> np.ndarray:
    try:
        base = embodiment._base
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robots = [base.scene[name] for name in ROBOTS]
    except (AttributeError, KeyError, TypeError) as err:
        raise RuntimeError(f"garments_on_hangers can't read the robot state from the embodiment: {err!r}") from err
    points = []
    for robot in robots:
        names = list(robot.joint_names)
        missing = sorted(set(ARM_JOINTS + FINGER_JOINTS) - set(names))
        if missing:
            raise RuntimeError(f"garments_on_hangers: the robot has no joints {missing}")
        q = robot.data.joint_pos.torch[0].detach().cpu().numpy().astype(float)
        root = robot.data.root_link_pose_w.torch[0].detach().cpu().numpy().astype(float)
        to_world = np.eye(4)
        to_world[:3, :3] = quat_xyzw_to_matrix(root[3:7])
        to_world[:3, 3] = root[:3] - origin
        links = forward_kinematics(YAM_URDF, dict(zip(names, q.tolist(), strict=True)))
        for link, tip in FINGERTIPS.items():
            points.append((to_world @ links[link] @ np.r_[tip, 1.0])[:3])
    return np.array(points)


def distance_to(points: np.ndarray, state, spec) -> np.ndarray:
    local = (np.asarray(points) - np.asarray(state.pos)) @ _rot(state.quat_xyzw)
    excess = np.maximum(np.abs(local) - np.asarray(spec.size) / 2, 0.0)
    return np.linalg.norm(excess, axis=1)


def released(objects, tips: np.ndarray) -> bool:
    return all(float(np.min(distance_to(tips, objects[n], s))) >= RELEASED for n, s in SPECS.items())


# -- hooks ---------------------------------------------------------------------------------------------------


def score(objects):
    """0 if any garment fell off the table. Otherwise 0.9 x max(0, C/3 - W/3): C garments at their right place,
    W at a wrong one (the other hanger, or the shelf). 1.0 comes from success() holding for 1 s."""
    t = tally(objects)
    if t["fell"]:
        return 0.0
    return PARTIAL_CAP * max(0.0, (len(t["correct"]) - len(t["wrong"])) / len(SPECS))


def all_done(objects) -> bool:
    t = tally(objects)
    return not t["fell"] and len(t["correct"]) == len(SPECS)


def success(objects):
    """All three garments at their right place, all still, every fingertip at least 3 cm from every garment.
    Held for SUCCESS_HOLD_S, this ends the trial with 1.0."""
    if not all_done(objects) or not all(objects[n].resting() for n in SPECS):
        return False
    return released(objects, fingertip_positions(_embodiment()))


def _goal(name: str) -> np.ndarray:
    s = SPECS[name]
    if DEST[name] == "shelf":
        return np.array([*SHELF_CENTRE, SHELF_TOP + s.size[2] / 2])
    return np.array([HANGER_X, HANGERS[DEST[name]][0], TOP + BAR_TOP_H + s.size[2] / 2])


def distance(objects):
    """Metres, summed over the garments not at their right place, from each to its place."""
    return float(sum(np.linalg.norm(np.asarray(objects[n].pos) - _goal(n))
                     for n, s in SPECS.items() if where(objects[n], s) != DEST[n]))


# -- demo ----------------------------------------------------------------------------------------------------
#
# The garments lie along x and the fingers close along y, across a garment's width. A garment going onto a bar is
# held 4 cm back from its middle (towards the robot), so the fingers stay clear of the bar while its middle is laid
# on it.

YAW = {"left": 0.0, "right": 0.0}
FINGER_REACH = 0.0135
MIN_TCP = TOP + FINGER_REACH + 0.0005
TRAVEL_Z = TOP + BAR_TOP_H + 0.06
SQUEEZE = 0.003
CLOSE_S, GRIP_SETTLE_S, OPEN_SETTLE_S = 1.5, 0.6, 0.5
REACH_FIX = {"right": (0.003, 0.0025), "left": (0.003, -0.0025)}
GRIP_BACK = {"trousers": 0.04, "silk_blouse": 0.04, "wool_sweater": 0.0}  # grip this far behind the middle (x)
DROP = 0.003
PLACE_FIX = {"trousers": (0.0, 0.0), "silk_blouse": (0.0, 0.0), "wool_sweater": (0.0, 0.0)}  # measured in sim


def _fingers(width: float) -> tuple[float, float]:
    return FINGER_OPEN, min(FINGER_CLOSED, -(width / 2 - SQUEEZE))


def _put(arm: str, name: str) -> list[Waypoint]:
    spec = ITEMS[name]
    fx, fy = REACH_FIX[arm]
    back = GRIP_BACK[name]
    centre_z = spec.pos[2] - SPAWN_GAP
    tcp = max(centre_z, MIN_TCP)
    below = tcp - (centre_z - spec.size[2] / 2)  # TCP above the garment's underside
    px, py = spec.pos[0] - back + fx, spec.pos[1] + fy
    goal = _goal(name)
    gx, gy = PLACE_FIX[name]
    qx, qy = goal[0] - back + fx + gx, goal[1] + fy + gy
    surface = goal[2] - spec.size[2] / 2
    qz = surface + DROP + below
    ajar, grip = _fingers(spec.size[1])
    y = YAW[arm]
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), ajar, f"to the {name}", y),
        Waypoint(2.0, (px, py, tcp), ajar, "descend", y),
        Waypoint(1.5, (px, py, tcp), ajar, "settle at the bottom", y),
        Waypoint(CLOSE_S, (px, py, tcp), grip, "close slowly", y),
        Waypoint(GRIP_SETTLE_S, (px, py, tcp), grip, "grip settle", y),
        Waypoint(1.5, (px, py, TRAVEL_Z), grip, "lift", y),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), grip, "over its place", y),
        Waypoint(1.5, (qx, qy, qz), grip, "lower onto it", y),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", y),
        Waypoint(0.6, (qx, qy, qz), ajar, "release", y),
        Waypoint(OPEN_SETTLE_S, (qx, qy, qz), ajar, "fingers clear", y),
        Waypoint(1.2, (qx - 0.03, qy, TRAVEL_Z), ajar, "back away and up", y),
    ]


PARK = {"left": (0.14, 0.27, TOP + 0.14), "right": (0.14, -0.27, TOP + 0.14)}


def _park(arm: str, seconds: float, label: str = "park") -> Waypoint:
    return Waypoint(seconds, PARK[arm], FINGER_OPEN, label, YAW[arm])


def _span(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    """The left arm lays the trousers over the trouser hanger while the right arm lays the silk blouse over the
    padded hanger; then the left arm lays the sweater flat on the shelf."""
    left = [*_put("left", "trousers"), *_put("left", "wool_sweater"), _park("left", 1.5)]
    right = [*_put("right", "silk_blouse"), _park("right", 1.5)]
    hold = max(_span(left), _span(right)) + 2.0
    left.append(_park("left", hold - _span(left), "hold still"))
    right.append(_park("right", hold - _span(right), "hold still"))
    return {"left": left, "right": right}
