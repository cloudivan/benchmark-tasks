"""On the station table: a small wardrobe (fixed: two side walls, a back wall, a middle divider and a shelf in each
half) stands at the back of the work area, open towards the arms. Two sliding doors close its front (loose, each
standing in its own floor track): the front door covers the left half and the rear door covers the right half;
each can slide across behind or in front of the other, so only one half can be open at a time. A folded wool
sweater lies on the table to the left.

  seen from above (y in cm, x forward)
     x 42 +---------------- back wall -----------------+
          |   left shelf    |div|    right shelf        |
     x 31 |  (y +0.5..+13.5)|   |  (y -0.5..-13.5)      |
   x 29.2 |                 rear door  (y -13.5..+1)    |      rear track
     x 25 |  front door (y -1..+13.5)                   |      front track
         side wall y +14                         side wall y -14
  sweater on the table at x 16, y +19

The sweater goes on the left shelf, so the front door has to be slid to the right first and slid back afterwards.
The wardrobe has no top, so that top-down grippers fit; the sweater has to go in through the opening, so passing
it over a door or a wall at any time scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=wardrobe_sliding_doors.py -E object_jitter=0 \\
           --scorer task_score --instruction "Put the sweater on the shelf in the left half of the wardrobe ..." ...
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

# -- fixed: tracks and the wardrobe -----------------------------------------------------------------------------

DOOR = (0.012, 0.145, 0.12)  # x thickness, y width, z height
DOOR_MASS = 0.20
TRACK_GAP = 0.002  # between a door and each of its rails
RAIL_T, RAIL_H = 0.005, 0.015
INNER_Y = 0.135  # the side walls' inner faces at y = +-13.5 cm
FRONT_X = 0.25  # front track centre
DOOR_SPACING = 0.03  # between the two doors' facing sides, as in a two-channel bypass track
_step = DOOR[0] / 2 + TRACK_GAP + RAIL_T / 2
REAR_X = FRONT_X + DOOR[0] + DOOR_SPACING  # rear track centre (29.2 cm)
RAIL_XS = (FRONT_X - _step, FRONT_X + _step, REAR_X - _step, REAR_X + _step)
WALL_T, WALL_H = 0.01, 0.16
WALL_FRONT_X, BACK_X = 0.232, 0.42  # side walls run from x 23.2 to the back wall's front face at 42
DIVIDER_FRONT_X = 0.31
SHELF_TOP_H = 0.05
WOOD, RAIL_GREY, SHELF_WHITE = (0.74, 0.62, 0.47), (0.55, 0.56, 0.58), (0.95, 0.95, 0.93)

_side_len = BACK_X + WALL_T - WALL_FRONT_X
WALLS = {  # name: Box (all fixed)
    "wall_left": Box("wall_left", pos=(WALL_FRONT_X + _side_len / 2, INNER_Y + WALL_T / 2, TOP + WALL_H / 2),
                     size=(_side_len, WALL_T, WALL_H), color=WOOD),
    "wall_right": Box("wall_right", pos=(WALL_FRONT_X + _side_len / 2, -INNER_Y - WALL_T / 2, TOP + WALL_H / 2),
                      size=(_side_len, WALL_T, WALL_H), color=WOOD),
    "wall_back": Box("wall_back", pos=(BACK_X + WALL_T / 2, 0.0, TOP + WALL_H / 2),
                     size=(WALL_T, 2 * (INNER_Y + WALL_T), WALL_H), color=WOOD),
    "divider": Box("divider", pos=((DIVIDER_FRONT_X + BACK_X) / 2, 0.0, TOP + WALL_H / 2),
                   size=(BACK_X - DIVIDER_FRONT_X, WALL_T, WALL_H), color=WOOD),
}
_shelf_w = INNER_Y - WALL_T / 2
SHELVES = {
    "left": Box("shelf_left", pos=((DIVIDER_FRONT_X + BACK_X) / 2, WALL_T / 2 + _shelf_w / 2, TOP + SHELF_TOP_H / 2),
                size=(BACK_X - DIVIDER_FRONT_X, _shelf_w, SHELF_TOP_H), color=SHELF_WHITE),
    "right": Box("shelf_right", pos=((DIVIDER_FRONT_X + BACK_X) / 2, -WALL_T / 2 - _shelf_w / 2,
                                     TOP + SHELF_TOP_H / 2),
                 size=(BACK_X - DIVIDER_FRONT_X, _shelf_w, SHELF_TOP_H), color=SHELF_WHITE),
}
RAILS = [Box(f"rail_{i}", pos=(x, 0.0, TOP + RAIL_H / 2), size=(RAIL_T, 2 * INNER_Y, RAIL_H), color=RAIL_GREY)
         for i, x in enumerate(RAIL_XS)]

# -- loose: the two doors and the sweater ------------------------------------------------------------------------

FRONT_CLOSED_Y = INNER_Y - DOOR[1] / 2  # +6.25 cm: covers y -1 .. +13.5
REAR_CLOSED_Y = -FRONT_CLOSED_Y  # covers y -13.5 .. +1
DOORS = {
    "front_door": Box("front_door", pos=(FRONT_X, FRONT_CLOSED_Y, TOP + DOOR[2] / 2 + SPAWN_GAP), size=DOOR,
                      mass=DOOR_MASS, color=(0.86, 0.80, 0.70)),
    "rear_door": Box("rear_door", pos=(REAR_X, REAR_CLOSED_Y, TOP + DOOR[2] / 2 + SPAWN_GAP), size=DOOR,
                     mass=DOOR_MASS, color=(0.80, 0.73, 0.62)),
}
TRACK_X = {"front_door": FRONT_X, "rear_door": REAR_X}
CLOSED_Y = {"front_door": FRONT_CLOSED_Y, "rear_door": REAR_CLOSED_Y}

SWEATER_SIZE = (0.08, 0.05, 0.025)
SWEATER = Box("sweater", pos=(0.16, 0.19, TOP + SWEATER_SIZE[2] / 2 + SPAWN_GAP), size=SWEATER_SIZE, mass=0.12,
              color=(0.30, 0.42, 0.33))

OBJECTS = [*WALLS.values(), *SHELVES.values(), *RAILS, *DOORS.values(), SWEATER]
SPECS = {**DOORS, "sweater": SWEATER}

# Scoring thresholds (m, rad).
DROPPED = 0.05
CLOSED_TOL = 0.01  # a door within this of its closed position (y)
IN_TRACK_X = 0.004  # a door's centre within this of its track's centre line, upright, standing on the table
UPRIGHT_TOL = np.radians(10.0)
REST_TOL = 0.005
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


def fixed_extent(box) -> tuple[np.ndarray, np.ndarray]:
    c, h = np.asarray(box.pos), np.asarray(box.size) / 2
    return c - h, c + h


def tilt(state) -> float:
    return float(np.arccos(np.clip(abs(_rot(state.quat_xyzw)[2, 2]), -1.0, 1.0)))


def fell(state, spec) -> bool:
    return bool(extent(state, spec)[0][2] < TOP - DROPPED)


def in_track(state, name: str) -> bool:
    """Upright (tilt under 10 degrees), standing on the table (underside within 5 mm) and centred on its track
    (within 4 mm in x)."""
    lo, _ = extent(state, DOORS[name])
    return bool(tilt(state) <= UPRIGHT_TOL and abs(lo[2] - TOP) <= REST_TOL
                and abs(state.pos[0] - TRACK_X[name]) <= IN_TRACK_X)


def closed(state, name: str) -> bool:
    return in_track(state, name) and abs(state.pos[1] - CLOSED_Y[name]) <= CLOSED_TOL


def on_shelf(state, side: str) -> bool:
    """Lying flat (tilt under 10 degrees), underside within 5 mm of the shelf top, wholly over the shelf."""
    lo, hi = extent(state, SWEATER)
    slo, shi = fixed_extent(SHELVES[side])
    return bool(tilt(state) <= FLAT_TOL and abs(lo[2] - shi[2]) <= REST_TOL
                and lo[0] >= slo[0] and hi[0] <= shi[0] and lo[1] >= slo[1] and hi[1] <= shi[1])


def _overlap_xy(a, b) -> bool:
    (alo, ahi), (blo, bhi) = a, b
    return bool(alo[0] < bhi[0] and ahi[0] > blo[0] and alo[1] < bhi[1] and ahi[1] > blo[1])


def over_blocker(objects) -> str | None:
    """The door or wall the sweater is over (its footprint overlapping the door's or wall's, at any height), if any.
    The wardrobe has no top, so this stands in for its roof: a sweater can only reach a shelf through an opening."""
    s = extent(objects["sweater"], SWEATER)
    for name, box in WALLS.items():
        if _overlap_xy(s, fixed_extent(box)):
            return name
    for name, spec in DOORS.items():
        if _overlap_xy(s, extent(objects[name], spec)):
            return name
    return None


# -- the robot (release check, episode clock) ---------------------------------------------------------------------

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
    raise RuntimeError("wardrobe_sliding_doors scores only inside a yam_arena embodiment's step "
                       "(inspect-robots run --embodiment yam_arena)")


def fingertip_positions(embodiment) -> np.ndarray:
    try:
        base = embodiment._base
        origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robots = [base.scene[name] for name in ROBOTS]
    except (AttributeError, KeyError, TypeError) as err:
        raise RuntimeError(f"wardrobe_sliding_doors can't read the robot state from the embodiment: {err!r}") from err
    points = []
    for robot in robots:
        names = list(robot.joint_names)
        missing = sorted(set(ARM_JOINTS + FINGER_JOINTS) - set(names))
        if missing:
            raise RuntimeError(f"wardrobe_sliding_doors: the robot has no joints {missing}")
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


# -- per-trial record ----------------------------------------------------------------------------------------
#
# Passing the sweater over a door or a wall at any time fails the trial. The record restarts when the embodiment's
# episode clock goes back (every reset sets it to 0).

_TRIAL = {"t": None, "failed": None, "tip_gap": None}


def _update(objects) -> dict:
    embodiment = _embodiment()
    t = float(embodiment._t)
    if _TRIAL["t"] is None or t < _TRIAL["t"]:
        _TRIAL.update(failed=None)
    elif t == _TRIAL["t"]:
        return _TRIAL
    _TRIAL["t"] = t
    tips = fingertip_positions(embodiment)
    _TRIAL["tip_gap"] = min(float(np.min(distance_to(tips, objects[n], s))) for n, s in SPECS.items())
    if _TRIAL["failed"] is None:
        blocker = over_blocker(objects)
        if blocker:
            _TRIAL["failed"] = f"sweater passed over the {blocker}"
        else:
            for n, s in SPECS.items():
                if fell(objects[n], s):
                    _TRIAL["failed"] = f"{n} fell off the table"
    return _TRIAL


def hard_zero(objects, trial) -> str | None:
    if trial["failed"]:
        return trial["failed"]
    for name in DOORS:
        if not in_track(objects[name], name):
            return f"{name} out of its track"
    return None


def stage(objects) -> float:
    """0.45 with the sweater on the left shelf; 0.9 when both doors are closed as well."""
    if not on_shelf(objects["sweater"], "left"):
        return 0.0
    return 0.9 if all(closed(objects[n], n) for n in DOORS) else 0.45


# -- hooks ---------------------------------------------------------------------------------------------------


def score(objects):
    """0 if the sweater ever passed over a door or a wall, anything fell off the table, or a door ends out of its
    track. Otherwise 0.45 with the sweater lying flat and wholly on the left shelf, 0.9 with both doors closed as
    well (each within 1 cm of its closed position). 1.0 comes from success() holding for 1 s."""
    trial = _update(objects)
    return 0.0 if hard_zero(objects, trial) else stage(objects)


def success(objects):
    """The sweater on the left shelf, both doors closed, everything still, every fingertip at least 3 cm from the
    sweater and the doors, with no failure so far. Held for SUCCESS_HOLD_S, this ends the trial with 1.0."""
    trial = _update(objects)
    if hard_zero(objects, trial) or stage(objects) < PARTIAL_CAP:
        return False
    return bool(all(objects[n].resting() for n in SPECS) and trial["tip_gap"] >= RELEASED)


def distance(objects):
    """Metres: the sweater's distance to the middle of the left shelf top, plus each door's distance from closed."""
    goal = np.array([SHELVES["left"].pos[0], SHELVES["left"].pos[1], TOP + SHELF_TOP_H + SWEATER_SIZE[2] / 2])
    d = float(np.linalg.norm(np.asarray(objects["sweater"].pos) - goal))
    return d + sum(abs(objects[n].pos[1] - CLOSED_Y[n]) for n in DOORS)


# -- demo ----------------------------------------------------------------------------------------------------
#
# The right arm slides the front door: it grips the door's top edge across its 1.2 cm thickness (fingers closing
# along x, yaw 90 deg, opened only 1.2 cm wider than the door on each side, so the back finger stays in the 3 cm
# between the doors) 3.25 cm right of the door's centre, and drags it 12.5 cm to the right. The left arm then carries the sweater round in front of the wardrobe and in through the left opening onto
# the shelf. Then the right arm drags the front door back.

FINGER_REACH = 0.0135
MIN_TCP = TOP + FINGER_REACH + 0.0005
TRAVEL_Z = TOP + 0.10
SQUEEZE = 0.003
REACH_FIX = {"right": (0.003, 0.0025), "left": (0.003, -0.0025)}
DOOR_GRIP_DEPTH = 0.02  # fingertips this far below the door's top edge
DOOR_GRIP_FROM_CENTRE = -0.0325  # grip point along y from the door's centre (towards its right end)
DOOR_TRAVEL = -0.125  # the front door slides this far (y) to open the left half, until it meets the right wall
OVERDRIVE = 0.015  # the fingers go this much further, so the door ends against the wall despite slip in the grip
DOOR_FIX = (0.0, 0.0)  # measured in sim
SWEATER_FIX = (0.0, 0.0)  # measured in sim
YAW_DOOR, YAW_SWEATER = np.pi / 2, 0.0
NARROW = -0.02  # fingers 2 cm either side of the TCP: clear of the opened door's end while backing out
CLEAR_Z = TOP + DOOR[2] + FINGER_REACH + 0.035  # fingertips 3.5 cm above the doors' tops


DOOR_PRE_OPEN = 0.012


def _grip(width: float) -> float:
    return min(FINGER_CLOSED, -(width / 2 - SQUEEZE))


def _slide_door(start_y: float, travel: float, label: str) -> list[Waypoint]:
    fx, fy = REACH_FIX["right"]
    dx, dy = DOOR_FIX
    x = FRONT_X + fx + dx
    y0 = start_y + DOOR_GRIP_FROM_CENTRE + fy + dy
    y1 = y0 + travel + np.sign(travel) * OVERDRIVE
    z = TOP + DOOR[2] - DOOR_GRIP_DEPTH + FINGER_REACH
    above = TOP + DOOR[2] + 0.06
    g, w = _grip(DOOR[0]), YAW_DOOR
    ajar = -(DOOR[0] / 2 + DOOR_PRE_OPEN)
    return [
        Waypoint(1.5, (x, y0, above), ajar, f"over the front door ({label})", w),
        Waypoint(1.5, (x, y0, z), ajar, "down over its top edge", w),
        Waypoint(1.0, (x, y0, z), ajar, "settle", w),
        Waypoint(1.2, (x, y0, z), g, "close on the edge", w),
        Waypoint(0.5, (x, y0, z), g, "grip settle", w),
        Waypoint(3.5, (x, y1, z), g, f"slide the door {label}", w),
        Waypoint(0.5, (x, y1, z), g, "hold", w),
        Waypoint(0.6, (x, y1, z), ajar, "release", w),
        Waypoint(0.4, (x, y1, z), ajar, "fingers clear", w),
        Waypoint(1.0, (x, y1, above), ajar, "rise", w),
    ]


def _sweater() -> list[Waypoint]:
    fx, fy = REACH_FIX["left"]
    sx, sy = SWEATER_FIX
    centre_z = SWEATER.pos[2] - SPAWN_GAP
    tcp = max(centre_z, MIN_TCP)
    below = tcp - (centre_z - SWEATER_SIZE[2] / 2)
    px, py = SWEATER.pos[0] + fx, SWEATER.pos[1] + fy
    gx, gy = SHELVES["left"].pos[0] + fx + sx, SHELVES["left"].pos[1] + fy + sy
    gz = TOP + SHELF_TOP_H + 0.003 + below
    front_x = 0.19 + fx  # in front of the wardrobe, clear of the side wall
    g, w = _grip(SWEATER_SIZE[1]), YAW_SWEATER
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "to the sweater", w),
        Waypoint(2.0, (px, py, tcp), FINGER_OPEN, "descend", w),
        Waypoint(1.5, (px, py, tcp), FINGER_OPEN, "settle", w),
        Waypoint(1.5, (px, py, tcp), g, "close slowly", w),
        Waypoint(0.6, (px, py, tcp), g, "grip settle", w),
        Waypoint(1.2, (px, py, TRAVEL_Z), g, "lift", w),
        Waypoint(2.0, (front_x, gy, TRAVEL_Z), g, "round to the left opening", w),
        Waypoint(2.0, (gx, gy, TRAVEL_Z), g, "in through the opening", w),
        Waypoint(1.2, (gx, gy, gz), g, "lower onto the shelf", w),
        Waypoint(0.6, (gx, gy, gz), g, "settle", w),
        Waypoint(0.6, (gx, gy, gz), FINGER_OPEN, "release", w),
        Waypoint(0.5, (gx, gy, gz), FINGER_OPEN, "fingers clear", w),
        Waypoint(0.8, (gx, gy, gz + 0.035), FINGER_OPEN, "rise clear of the sweater", w),
        Waypoint(0.6, (gx, gy, gz + 0.035), NARROW, "narrow the fingers", w),
        Waypoint(1.2, (gx, gy, CLEAR_Z), NARROW, "rise above the doors", w),
        Waypoint(2.0, (front_x, gy, CLEAR_Z), NARROW, "back out", w),
    ]


PARK = {"left": (0.14, 0.27, TOP + 0.14), "right": (0.14, -0.27, TOP + 0.14)}


def _park(arm: str, seconds: float, label: str, yaw: float) -> Waypoint:
    return Waypoint(seconds, PARK[arm], FINGER_OPEN, label, yaw)


def _span(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    """Right arm: open the front door, park while the left arm shelves the sweater, then close the door."""
    open_door = _slide_door(FRONT_CLOSED_Y, DOOR_TRAVEL, "right")
    close_door = _slide_door(FRONT_CLOSED_Y + DOOR_TRAVEL, -DOOR_TRAVEL, "back")
    shelve = _sweater()
    right = [*open_door, _park("right", 1.5, "park", YAW_DOOR)]
    left = [_park("left", _span(right), "wait for the door", YAW_SWEATER), *shelve,
            _park("left", 1.5, "park", YAW_SWEATER)]
    right.append(_park("right", _span(left) - _span(right), "wait for the sweater", YAW_DOOR))
    right += [*close_door, _park("right", 1.5, "park", YAW_DOOR)]
    hold = max(_span(left), _span(right)) + 2.0
    left.append(_park("left", hold - _span(left), "hold still", YAW_SWEATER))
    right.append(_park("right", hold - _span(right), "hold still", YAW_DOOR))
    return {"left": left, "right": right}
