"""On the station table: a low chest (fixed: two side walls, a back wall and a top) stands at the back of the work
area with its opening towards the arms. Its drawer (loose, a solid block standing for the drawer's floor) is pulled
8 cm out of it. The gap between the drawer and the chest's top is 2.6 cm: room for one layer of rolled socks. Three
rolled pairs of socks (loose) lie on the table, two on the left and one on the right.

  seen from above (x forward, y in cm)
     x 38  +--- back wall ---+
           |   chest, top    |        side walls' inner faces at y +-8.3; inside 12 cm deep
     x 26  +--- opening -----+        the drawer's front face is at x 15 (closed: x 23, its 3 cm front standing proud)
     x 15     [ drawer ]              drawer 15 (x) x 16 (y) x 2 cm, y -8..+8
  socks:  x 15 and 21 at y +17 (left);  x 17 at y -17 (right)

Put the socks on the drawer and push it shut. Socks stacked or stood on end are taller than the gap and are
scraped off on the chest's front edge as the drawer goes in, as in an overfull drawer. The drawer's front 3 cm stay
outside the chest when it is closed, like a drawer's front panel, so socks left there are not in the drawer.

Run it:  inspect-robots run --embodiment yam_arena -E task=sock_drawer.py -E object_jitter=0 \\
           --scorer task_score --instruction "Put the three pairs of socks in the drawer and close it." ...
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

# -- fixed: the chest ------------------------------------------------------------------------------------------

DRAWER_SIZE = (0.15, 0.16, 0.02)  # x depth, y width, z height
OPEN_BY = 0.08  # the drawer starts pulled out this far
CHEST_FRONT_X = 0.26
INNER_DEPTH = 0.12
CHEST_BACK_X = CHEST_FRONT_X + INNER_DEPTH  # the back wall's front face: a closed drawer's back touches it
FRONT_PROUD = DRAWER_SIZE[0] - INNER_DEPTH  # 3 cm of a closed drawer stand out of the chest, like a front panel
SIDE_GAP = 0.003  # between the drawer and each side wall
CLEARANCE = 0.026  # between the drawer's top and the underside of the chest's top
INNER_H = DRAWER_SIZE[2] + CLEARANCE
WALL_T, TOP_T = 0.01, 0.012
CHEST_COLOUR = (0.70, 0.56, 0.40)
_inner_y = DRAWER_SIZE[1] / 2 + SIDE_GAP
_depth = CHEST_BACK_X + WALL_T - CHEST_FRONT_X
CHEST = [
    Box("chest_side_left", pos=(CHEST_FRONT_X + _depth / 2, _inner_y + WALL_T / 2, TOP + INNER_H / 2),
        size=(_depth, WALL_T, INNER_H), color=CHEST_COLOUR),
    Box("chest_side_right", pos=(CHEST_FRONT_X + _depth / 2, -_inner_y - WALL_T / 2, TOP + INNER_H / 2),
        size=(_depth, WALL_T, INNER_H), color=CHEST_COLOUR),
    Box("chest_back", pos=(CHEST_BACK_X + WALL_T / 2, 0.0, TOP + INNER_H / 2),
        size=(WALL_T, 2 * _inner_y, INNER_H), color=CHEST_COLOUR),
    Box("chest_top", pos=(CHEST_FRONT_X + _depth / 2, 0.0, TOP + INNER_H + TOP_T / 2),
        size=(_depth, 2 * (_inner_y + WALL_T), TOP_T), color=CHEST_COLOUR),
]

# -- loose: the drawer and three rolled pairs of socks --------------------------------------------------------------

CLOSED_X = CHEST_BACK_X - DRAWER_SIZE[0] / 2
CLOSED_FRONT_X = CHEST_BACK_X - DRAWER_SIZE[0]  # 23 cm
DRAWER = Box("drawer", pos=(CLOSED_X - OPEN_BY, 0.0, TOP + DRAWER_SIZE[2] / 2 + SPAWN_GAP), size=DRAWER_SIZE,
             mass=0.20, color=(0.86, 0.76, 0.60))
DRAWER_TOP = TOP + DRAWER_SIZE[2]

SOCK = (0.04, 0.03, 0.02)  # a rolled pair: 4 (x) x 3 (y) x 2 cm
SOCK_MASS = 0.03
SOCKS = {
    "socks_navy": Box("socks_navy", pos=(0.15, 0.17, TOP + SOCK[2] / 2 + SPAWN_GAP), size=SOCK, mass=SOCK_MASS,
                      color=(0.13, 0.17, 0.38)),
    "socks_grey": Box("socks_grey", pos=(0.21, 0.17, TOP + SOCK[2] / 2 + SPAWN_GAP), size=SOCK, mass=SOCK_MASS,
                      color=(0.55, 0.56, 0.58)),
    "socks_red": Box("socks_red", pos=(0.17, -0.17, TOP + SOCK[2] / 2 + SPAWN_GAP), size=SOCK, mass=SOCK_MASS,
                     color=(0.72, 0.15, 0.15)),
}

OBJECTS = [*CHEST, DRAWER, *SOCKS.values()]
SPECS = {"drawer": DRAWER, **SOCKS}

# Scoring thresholds (m, rad).
DROPPED = 0.05
REST_TOL = 0.005  # a sock's underside within this of the drawer's top
CLOSED_TOL = 0.005  # the drawer's back face within this of the back wall
FLAT_TOL = np.radians(10.0)
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


def tilt(state) -> float:
    return float(np.arccos(np.clip(abs(_rot(state.quat_xyzw)[2, 2]), -1.0, 1.0)))


def fell(state, spec) -> bool:
    return bool(extent(state, spec)[0][2] < TOP - DROPPED)


def on_drawer(objects, name: str) -> bool:
    """The sock rests on the drawer: its underside within 5 mm of the drawer's top and its whole footprint over it."""
    lo, hi = extent(objects[name], SOCKS[name])
    dlo, dhi = extent(objects["drawer"], DRAWER)
    return bool(abs(lo[2] - dhi[2]) <= REST_TOL and lo[0] >= dlo[0] and hi[0] <= dhi[0]
                and lo[1] >= dlo[1] and hi[1] <= dhi[1])


def drawer_closed(objects) -> bool:
    """Pushed home: the drawer's back face within 5 mm of the back wall, and lying flat."""
    d = objects["drawer"]
    _, hi = extent(d, DRAWER)
    return bool(abs(hi[0] - CHEST_BACK_X) <= CLOSED_TOL and tilt(d) <= FLAT_TOL)


def inside(objects, name: str) -> bool:
    """Wholly behind the chest's front (x 26), so inside the closed chest rather than on the drawer's front."""
    return bool(extent(objects[name], SOCKS[name])[0][0] >= CHEST_FRONT_X)


def tally(objects) -> dict:
    on = [n for n in SOCKS if on_drawer(objects, n)]
    return {
        "fell": [n for n, s in SPECS.items() if fell(objects[n], s)],
        "on_drawer": on,
        "inside": [n for n in on if inside(objects, n)],
        "closed": drawer_closed(objects),
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
    raise RuntimeError("sock_drawer checks release only inside a yam_arena embodiment's step "
                       "(inspect-robots run --embodiment yam_arena)")


def fingertip_positions(embodiment) -> np.ndarray:
    try:
        base = embodiment._base
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robots = [base.scene[name] for name in ROBOTS]
    except (AttributeError, KeyError, TypeError) as err:
        raise RuntimeError(f"sock_drawer can't read the robot state from the embodiment: {err!r}") from err
    points = []
    for robot in robots:
        names = list(robot.joint_names)
        missing = sorted(set(ARM_JOINTS + FINGER_JOINTS) - set(names))
        if missing:
            raise RuntimeError(f"sock_drawer: the robot has no joints {missing}")
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
    """0 if the drawer or a pair of socks fell off the table. Otherwise 0.9 x (0.2 N + 0.4 C): N = pairs resting on
    the drawer (0 to 3), C = 1 if the drawer is closed with all three on it and inside the chest. Closing the drawer
    on a stacked or upright pair scrapes it off on the chest's front edge, so that pair no longer counts and C stays
    0. 1.0 comes from success() holding for 1 s."""
    t = tally(objects)
    if t["fell"]:
        return 0.0
    n = len(t["on_drawer"])
    c = 1.0 if t["closed"] and len(t["inside"]) == len(SOCKS) else 0.0
    return PARTIAL_CAP * (0.2 * n + 0.4 * c)


def all_done(objects) -> bool:
    t = tally(objects)
    return not t["fell"] and t["closed"] and len(t["inside"]) == len(SOCKS)


def success(objects):
    """All three pairs on the drawer and inside the chest, the drawer closed, everything still, every fingertip at least 3 cm from the
    drawer and the socks. Held for SUCCESS_HOLD_S, this ends the trial with 1.0."""
    if not all_done(objects) or not all(objects[n].resting() for n in SPECS):
        return False
    return released(objects, fingertip_positions(_embodiment()))


def distance(objects):
    """Metres: each pair not on the drawer, to the drawer's top centre, plus the drawer's distance from closed."""
    d = objects["drawer"]
    goal = np.array([d.pos[0], d.pos[1], DRAWER_TOP + SOCK[2] / 2])
    total = sum(float(np.linalg.norm(np.asarray(objects[n].pos) - goal)) for n in SOCKS if not on_drawer(objects, n))
    return total + abs(extent(d, DRAWER)[1][0] - CHEST_BACK_X)


# -- demo ----------------------------------------------------------------------------------------------------
#
# Each pair lies with its long side along x; the fingers close along y across its 3 cm width. The left arm lays the
# grey pair in the middle of the drawer's exposed part and the navy pair 5.5 cm to its left (2.5 cm apart), the right
# arm lays the red pair 5.5 cm to the right, then pushes the drawer shut with its closed fingers against the drawer's
# front face. Releases open the fingers only 8 mm wider than a pair, so they don't push its neighbours.

YAW = {"left": 0.0, "right": 0.0}
FINGER_REACH = 0.0135
MIN_TCP = TOP + FINGER_REACH + 0.0005
TRAVEL_Z = TOP + INNER_H + TOP_T + 0.06
SQUEEZE = 0.003
REACH_FIX = {"right": (0.003, 0.0025), "left": (0.003, -0.0025)}
SLOT_X = CLOSED_FRONT_X - OPEN_BY + 0.06  # 6 cm in from the drawer's front face: inside the chest once closed
SLOTS = {"socks_navy": (SLOT_X, 0.055), "socks_grey": (SLOT_X, 0.0), "socks_red": (SLOT_X, -0.055)}
RELEASE = -(SOCK[1] / 2 + 0.008)
PLACE_FIX = {n: (0.0, 0.0) for n in SOCKS}  # measured in sim
PUSH_FIX = (0.01, 0.0)  # pushes 1 cm past closed: the back wall stops the drawer there
FINGER_PAD = 0.012  # the closed fingers' extent in front of the TCP along x (approx.)
PUSH_Z = TOP + FINGER_REACH + 0.008  # fingertips 8 mm above the table, mid-height on the drawer's front face


def _put(arm: str, name: str) -> list[Waypoint]:
    spec = SOCKS[name]
    fx, fy = REACH_FIX[arm]
    centre_z = spec.pos[2] - SPAWN_GAP
    tcp = max(centre_z, MIN_TCP)
    below = tcp - (centre_z - spec.size[2] / 2)
    px, py = spec.pos[0] + fx, spec.pos[1] + fy
    sx, sy = SLOTS[name]
    gx, gy = PLACE_FIX[name]
    qx, qy, qz = sx + fx + gx, sy + fy + gy, DRAWER_TOP + 0.003 + below
    grip = min(FINGER_CLOSED, -(spec.size[1] / 2 - SQUEEZE))
    w = YAW[arm]
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, f"to the {name}", w),
        Waypoint(2.0, (px, py, tcp), FINGER_OPEN, "descend", w),
        Waypoint(1.5, (px, py, tcp), FINGER_OPEN, "settle", w),
        Waypoint(1.5, (px, py, tcp), grip, "close slowly", w),
        Waypoint(0.6, (px, py, tcp), grip, "grip settle", w),
        Waypoint(1.2, (px, py, TRAVEL_Z), grip, "lift", w),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), grip, "over the drawer", w),
        Waypoint(1.2, (qx, qy, qz), grip, "lower onto it", w),
        Waypoint(0.5, (qx, qy, qz), grip, "settle", w),
        Waypoint(0.6, (qx, qy, qz), RELEASE, "release", w),
        Waypoint(0.5, (qx, qy, qz), RELEASE, "fingers clear", w),
        Waypoint(1.0, (qx, qy, TRAVEL_Z), RELEASE, "rise", w),
    ]


def _push() -> list[Waypoint]:
    fx, fy = REACH_FIX["right"]
    px, py = PUSH_FIX
    front = CLOSED_FRONT_X - OPEN_BY
    x0 = front - FINGER_PAD - 0.03 + fx  # 3 cm in front of the drawer
    x1 = CLOSED_FRONT_X - FINGER_PAD + 0.003 + fx + px  # past closed, so the back wall stops it
    y = 0.0 + fy + py
    w = YAW["right"]
    return [
        Waypoint(1.5, (x0, y, TRAVEL_Z), FINGER_CLOSED, "in front of the drawer", w),
        Waypoint(1.5, (x0, y, PUSH_Z), FINGER_CLOSED, "down to its front face", w),
        Waypoint(1.0, (x0, y, PUSH_Z), FINGER_CLOSED, "settle", w),
        Waypoint(4.0, (x1, y, PUSH_Z), FINGER_CLOSED, "push it shut, slowly", w),
        Waypoint(0.8, (x1, y, PUSH_Z), FINGER_CLOSED, "hold", w),
        Waypoint(1.0, (x1 - 0.05, y, PUSH_Z), FINGER_CLOSED, "back off", w),
        Waypoint(1.0, (x1 - 0.05, y, TRAVEL_Z), FINGER_OPEN, "rise", w),
    ]


PARK = {"left": (0.14, 0.27, TOP + 0.12), "right": (0.14, -0.27, TOP + 0.12)}


def _park(arm: str, seconds: float, label: str = "park") -> Waypoint:
    return Waypoint(seconds, PARK[arm], FINGER_OPEN, label, YAW[arm])


def _span(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    """Left arm: grey (middle) and navy pairs onto the drawer, then park. Right arm: waits parked until then (one arm over
    the drawer at a time), lays the red pair, then pushes the drawer shut."""
    left = [*_put("left", "socks_grey"), *_put("left", "socks_navy"), _park("left", 1.5)]
    right = [_park("right", _span(left), "wait for the left arm"), *_put("right", "socks_red"), *_push(),
             _park("right", 1.5)]
    hold = max(_span(left), _span(right)) + 2.0
    left.append(_park("left", hold - _span(left), "hold still"))
    right.append(_park("right", hold - _span(right), "hold still"))
    return {"left": left, "right": right}
