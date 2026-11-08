"""On the station table: a laundry pickup rack (fixed) stands across the back of the work area with three pairs of
hanger arms, each pair sized for one type of hanger: wide arms for a coat hanger on the right, middle arms for a
shirt hanger, narrow arms for a child's hanger on the left. Three hangers lie on the table in front of it, each
represented by its shoulder bar (loose): a dark wooden coat hanger, a white plastic shirt hanger and a small pink
child's hanger.

  seen from above (y in cm)          +20 narrow pair     +6 middle pair     -14 wide pair      (rack, x 30)
                                     +16 shirt bar        +4 child's bar     -10 coat bar       (table, x 16)

Each hanger is hung when its bar rests level on the cradles of the pair it fits, both ends overhanging that pair's
arms by no more than 1.5 cm. A smaller hanger falls between a wider pair's arms; a larger one overhangs a narrower
pair by more than 1.5 cm. So each hanger fits exactly one pair.

Run it:  inspect-robots run --embodiment yam_arena -E task=hangers_on_rack.py -E object_jitter=0 \\
           --scorer task_score --instruction "Hang each hanger on the pair of rack arms it fits: ..." ...
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

# -- fixed: the rack, three pairs of arms at x = 30 cm -------------------------------------------------------------
#
# Each pair's two arms stand one behind the other (along x), so a hanger's bar lies pointing away from the robot.
# Each arm is a post standing on the table with a cradle on top: two lips, left and right of the post's top face,
# that keep a bar from rolling off in y. A bar rests on the post tops between the lips.

RACK_X = 0.30
ARM_H = 0.10  # post top (cradle floor) this far above the table
POST = (0.015, 0.03)  # x, y footprint of each post
LIP = (0.015, 0.005, 0.012)  # x, y, z of each cradle lip
CRADLE_GAP = 0.02  # between the two lips of a cradle (y); the bars are 1.2 to 1.6 cm thick
ARM_GREY = (0.70, 0.71, 0.73)
CRADLE_TOP = TOP + ARM_H

# pair name: (centre y, centre-to-centre spacing of its two posts along x, about x = RACK_X)
PAIRS = {
    "narrow": (0.20, 0.060),  # inner gap 4.5 cm, outer span 7.5 cm
    "middle": (0.06, 0.100),  # inner gap 8.5 cm, outer span 11.5 cm
    "wide": (-0.14, 0.140),  # inner gap 12.5 cm, outer span 15.5 cm
}


def _arm(name: str, x: float, y: float) -> list[Box]:
    lip_z = CRADLE_TOP + LIP[2] / 2
    lip_dy = CRADLE_GAP / 2 + LIP[1] / 2
    return [
        Box(f"{name}_post", pos=(x, y, TOP + ARM_H / 2), size=(POST[0], POST[1], ARM_H), color=ARM_GREY),
        Box(f"{name}_lip_left", pos=(x, y + lip_dy, lip_z), size=LIP, color=ARM_GREY),
        Box(f"{name}_lip_right", pos=(x, y - lip_dy, lip_z), size=LIP, color=ARM_GREY),
    ]


def arm_xs(pair: str) -> tuple[float, float]:
    spacing = PAIRS[pair][1]
    return RACK_X - spacing / 2, RACK_X + spacing / 2


RACK = [b for p in PAIRS for i, x in enumerate(arm_xs(p)) for b in _arm(f"{p}_arm{i}", x, PAIRS[p][0])]

# -- loose: three hangers, each its shoulder bar, lying on the table centred at x = 16 cm, long side along x ---------------

ROW_X = 0.16
# hanger name: (bar size x, y, z; mass; colour; start y; the pair it fits)
HANGERS = {
    "coat_hanger": ((0.160, 0.016, 0.030), 0.15, (0.36, 0.22, 0.12), -0.10, "wide"),
    "shirt_hanger": ((0.120, 0.014, 0.026), 0.06, (0.95, 0.95, 0.93), 0.16, "middle"),
    "child_hanger": ((0.080, 0.012, 0.022), 0.03, (0.93, 0.55, 0.70), 0.04, "narrow"),
}
BARS = {
    n: Box(n, pos=(ROW_X, y, TOP + size[2] / 2 + SPAWN_GAP), size=size, mass=m, color=c)
    for n, (size, m, c, y, _) in HANGERS.items()
}
FITS = {n: h[4] for n, h in HANGERS.items()}

OBJECTS = [*RACK, *BARS.values()]
SPECS = dict(BARS)

# Scoring thresholds (m, rad).
DROPPED = 0.05
MAX_OVERHANG = 0.015  # each end of a hung bar overhangs its pair's arms by no more than this
CENTRE_TOL = 0.01  # a hung bar's centre within this of the pair's centre in y, and of the cradles' line in x
REST_TOL = 0.004  # a hung bar's underside within this of the cradle floor
ALIGN_TOL = np.radians(5.0)  # a hung bar's long axis within this of y, and level
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


def aligned(state) -> bool:
    """Long axis (the bar's local x) within ALIGN_TOL of the world x axis, in yaw and in tilt."""
    axis = _rot(state.quat_xyzw)[:, 0]
    return bool(np.arccos(np.clip(abs(axis[0]), -1.0, 1.0)) <= ALIGN_TOL)


def hung_on(state, spec) -> str | None:
    """The pair the bar is hung on, if any: level and along x, its centre within 1 cm of the pair's centre line (y)
    and of the pair's middle (x), its underside within 4 mm of the cradle floor, and each end overhanging the pair's
    arms by no more than 1.5 cm. A bar shorter than a pair's inner gap can't rest on it."""
    if not aligned(state):
        return None
    lo, _ = extent(state, spec)
    if abs(lo[2] - CRADLE_TOP) > REST_TOL or abs(state.pos[0] - RACK_X) > CENTRE_TOL:
        return None
    length = spec.size[0]
    for pair, (cy, spacing) in PAIRS.items():
        inner, outer = spacing - POST[0], spacing + POST[0]
        overhang = (length - outer) / 2
        if abs(state.pos[1] - cy) <= CENTRE_TOL and length > inner and overhang <= MAX_OVERHANG:
            return pair
    return None


def tally(objects) -> dict:
    where = {n: hung_on(objects[n], s) for n, s in SPECS.items()}
    return {
        "fell": [n for n, s in SPECS.items() if fell(objects[n], s)],
        "hung": [n for n, p in where.items() if p == FITS[n]],
        "where": where,
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
    raise RuntimeError("hangers_on_rack checks release only inside a yam_arena embodiment's step "
                       "(inspect-robots run --embodiment yam_arena)")


def fingertip_positions(embodiment) -> np.ndarray:
    try:
        base = embodiment._base
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robots = [base.scene[name] for name in ROBOTS]
    except (AttributeError, KeyError, TypeError) as err:
        raise RuntimeError(f"hangers_on_rack can't read the robot state from the embodiment: {err!r}") from err
    points = []
    for robot in robots:
        names = list(robot.joint_names)
        missing = sorted(set(ARM_JOINTS + FINGER_JOINTS) - set(names))
        if missing:
            raise RuntimeError(f"hangers_on_rack: the robot has no joints {missing}")
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
    """0 if any hanger fell off the table. Otherwise 0.9 x H/3, H = hangers hung on the pair they fit. 1.0 comes
    from success() holding for 1 s."""
    t = tally(objects)
    if t["fell"]:
        return 0.0
    return PARTIAL_CAP * len(t["hung"]) / len(SPECS)


def all_hung(objects) -> bool:
    t = tally(objects)
    return not t["fell"] and len(t["hung"]) == len(SPECS)


def success(objects):
    """All three hangers hung on the pairs they fit, all still, every fingertip at least 3 cm from every hanger.
    Held for SUCCESS_HOLD_S, this ends the trial with 1.0."""
    if not all_hung(objects) or not all(objects[n].resting() for n in SPECS):
        return False
    return released(objects, fingertip_positions(_embodiment()))


def distance(objects):
    """Metres, summed over the hangers not hung on their pair, from each bar's centre to its pair's centre at
    cradle height."""
    total = 0.0
    for n, s in SPECS.items():
        if hung_on(objects[n], s) != FITS[n]:
            goal = np.array([RACK_X, PAIRS[FITS[n]][0], CRADLE_TOP + s.size[2] / 2])
            total += float(np.linalg.norm(np.asarray(objects[n].pos) - goal))
    return total


# -- demo ----------------------------------------------------------------------------------------------------
#
# The bars lie along x, so the fingers close along y, across a bar's thickness, at the bar's middle.

YAW = {"left": 0.0, "right": 0.0}
FINGER_REACH = 0.0135
MIN_TCP = TOP + FINGER_REACH + 0.0005
TRAVEL_Z = TOP + ARM_H + 0.08  # a held bar clears the cradle lips by about 6 cm
SQUEEZE = 0.002
CLOSE_S, GRIP_SETTLE_S, OPEN_SETTLE_S = 1.5, 0.6, 0.4
REACH_FIX = {"right": (0.003, 0.0025), "left": (0.003, -0.0025)}
DROP = 0.003  # the bar's underside this far above the cradle floor when let go
PLACE_FIX = {"coat_hanger": (-0.008, 0.0), "shirt_hanger": (0.0, 0.0), "child_hanger": (-0.017, 0.0)}  # measured in sim


def _fingers(width: float) -> tuple[float, float]:
    """Fully open on the way down (the bars are thin, so this catches a bar a few cm off the aim; the closing
    fingers push it to the middle), then a bounded squeeze."""
    return FINGER_OPEN, min(FINGER_CLOSED, -(width / 2 - SQUEEZE))


def _hang(arm: str, name: str) -> list[Waypoint]:
    spec = BARS[name]
    fx, fy = REACH_FIX[arm]
    centre_z = spec.pos[2] - SPAWN_GAP
    tcp = max(centre_z, MIN_TCP)
    below = tcp - (centre_z - spec.size[2] / 2)  # TCP above the bar's underside
    px, py = spec.pos[0] + fx, spec.pos[1] + fy
    qx, qy = RACK_X + fx + PLACE_FIX[name][0], PAIRS[FITS[name]][0] + fy + PLACE_FIX[name][1]
    qz = CRADLE_TOP + DROP + below
    ajar, grip = _fingers(spec.size[1])
    y = YAW[arm]
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), ajar, f"to the {name}", y),
        Waypoint(2.0, (px, py, tcp), ajar, "descend", y),
        Waypoint(1.5, (px, py, tcp), ajar, "settle at the bottom", y),
        Waypoint(CLOSE_S, (px, py, tcp), grip, "close slowly", y),
        Waypoint(GRIP_SETTLE_S, (px, py, tcp), grip, "grip settle", y),
        Waypoint(1.5, (px, py, TRAVEL_Z), grip, "lift", y),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), grip, "over its arms", y),
        Waypoint(1.5, (qx, qy, qz), grip, "lower into the cradles", y),
        Waypoint(0.5, (qx, qy, qz), grip, "settle", y),
        Waypoint(0.6, (qx, qy, qz), ajar, "release", y),
        Waypoint(OPEN_SETTLE_S, (qx, qy, qz), ajar, "fingers clear", y),
        Waypoint(1.2, (qx, qy, TRAVEL_Z), ajar, "rise", y),
    ]


PARK = {"left": (0.14, 0.27, TOP + 0.12), "right": (0.14, -0.27, TOP + 0.12)}


def _park(arm: str, seconds: float, label: str = "park") -> Waypoint:
    return Waypoint(seconds, PARK[arm], FINGER_OPEN, label, YAW[arm])


def _span(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    """The right arm hangs the coat hanger on the wide pair while the left arm hangs the child's hanger on the
    narrow pair; then the left arm hangs the shirt hanger on the middle pair."""
    right = [*_hang("right", "coat_hanger"), _park("right", 1.5)]
    left = [*_hang("left", "child_hanger"), *_hang("left", "shirt_hanger"), _park("left", 1.5)]
    hold = max(_span(left), _span(right)) + 2.0
    left.append(_park("left", hold - _span(left), "hold still"))
    right.append(_park("right", hold - _span(right), "hold still"))
    return {"left": left, "right": right}
