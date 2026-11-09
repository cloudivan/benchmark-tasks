"""Offline checks of the scorer against made-up trials (no simulator): a stand-in embodiment supplies the episode
clock and puts the fingertips far away. Run: python test_scorer.py"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import wardrobe_sliding_doors as t  # noqa: E402

ID = (0.0, 0.0, 0.0, 1.0)
CLOCK = SimpleNamespace(_t=0.0)
t._embodiment = lambda: CLOCK
t.fingertip_positions = lambda emb: np.full((4, 3), 5.0)


def tiltq(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def st(pos, q=ID):
    return SimpleNamespace(pos=pos, quat_xyzw=q, resting=lambda: True)


def door(name, y, q=ID):
    return st((t.TRACK_X[name], y, t.TOP + t.DOOR[2] / 2), q)


def sweater_at(x, y, z=None):
    return st((x, y, z if z is not None else t.TOP + t.SWEATER_SIZE[2] / 2))


SHELF_L = sweater_at(t.SHELVES["left"].pos[0], t.SHELVES["left"].pos[1], t.TOP + t.SHELF_TOP_H + 0.0125)
SHELF_R = sweater_at(t.SHELVES["right"].pos[0], t.SHELVES["right"].pos[1], t.TOP + t.SHELF_TOP_H + 0.0125)
FRONT_OPEN = t.FRONT_CLOSED_Y + t.DOOR_TRAVEL
START = {"front_door": door("front_door", t.FRONT_CLOSED_Y), "rear_door": door("rear_door", t.REAR_CLOSED_Y),
         "sweater": st(t.SWEATER.pos)}
IN_FRONT = sweater_at(0.19, 0.07, t.TOP + 0.09)


def run(states):
    """Feed a trial step by step from a fresh episode; return the last score."""
    CLOCK._t = 0.0
    t.score(START)  # step 0: a new episode
    out = 0.0
    for i, s in enumerate(states, 1):
        CLOCK._t = 0.05 * i
        out = t.score(s)
    return out


def S(**kw):
    return {**START, **kw}


cases = {
    "nothing done": ([S()], 0.0),
    "proper: open, in through the opening, close": (
        [S(front_door=door("front_door", FRONT_OPEN)), S(front_door=door("front_door", FRONT_OPEN), sweater=IN_FRONT),
         S(front_door=door("front_door", FRONT_OPEN), sweater=SHELF_L), S(sweater=SHELF_L)], 0.9),
    "door left open": ([S(front_door=door("front_door", FRONT_OPEN)),
                        S(front_door=door("front_door", FRONT_OPEN), sweater=SHELF_L)], 0.45),
    "lifted over the closed front door": ([S(sweater=sweater_at(0.25, 0.07, t.TOP + 0.2)), S(sweater=SHELF_L)], 0.0),
    "lifted over the left side wall": ([S(sweater=sweater_at(0.33, 0.15, t.TOP + 0.2)), S(sweater=SHELF_L)], 0.0),
    "right half instead (rear door slid left)": (
        [S(rear_door=door("rear_door", t.REAR_CLOSED_Y - t.DOOR_TRAVEL)),
         S(rear_door=door("rear_door", t.REAR_CLOSED_Y - t.DOOR_TRAVEL), sweater=SHELF_R), S(sweater=SHELF_R)], 0.0),
    "front door knocked out of its track": ([S(front_door=door("front_door", FRONT_OPEN, tiltq(30)),
                                               sweater=SHELF_L)], 0.0),
    "sweater fell off the table": ([S(sweater=sweater_at(0.2, 0.2, t.TOP - 0.3))], 0.0),
}

ok = True
for name, (states, want) in cases.items():
    got = run(states)
    line = f"{name:48s} {got:.3f}  failed={t._TRIAL['failed']}"
    if abs(got - want) > 1e-6:
        ok = False
        line += f"  <-- expected {want:.3f}"
    print(line)

# A new episode clears an earlier failure.
run([S(sweater=sweater_at(0.25, 0.07, t.TOP + 0.2))])
assert t._TRIAL["failed"]
assert run([S(sweater=SHELF_L)]) == 0.9 and t._TRIAL["failed"] is None  # doors closed, no failure carried over
d = t.demo()
print("demo seconds:", round(max(sum(w.duration for w in d[a]) for a in d), 1))
print("ALL OK" if ok else "MISMATCH")
sys.exit(0 if ok else 1)
