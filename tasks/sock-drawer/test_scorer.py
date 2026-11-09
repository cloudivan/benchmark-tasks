"""Offline checks of the scorer against made-up outcomes (no simulator). Run: python test_scorer.py"""

import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))
import sock_drawer as t  # noqa: E402

ID = (0.0, 0.0, 0.0, 1.0)


def st(pos, q=ID):
    return SimpleNamespace(pos=pos, quat_xyzw=q, resting=lambda: True)


def drawer_at(x):
    return st((x, 0.0, t.TOP + t.DRAWER_SIZE[2] / 2))


OPEN_X, SHUT_X = t.DRAWER.pos[0], t.CLOSED_X


def sock_on(drawer_x, slot_y, dz=0.0, dx=0.0):
    """On the drawer at the demo's slot (6 cm in from its front face)."""
    front = drawer_x - t.DRAWER_SIZE[0] / 2
    return st((front + 0.06 + dx, slot_y, t.DRAWER_TOP + t.SOCK[2] / 2 + dz))


START = {"drawer": drawer_at(OPEN_X), **{n: st(s.pos) for n, s in t.SOCKS.items()}}
YS = {"socks_navy": 0.055, "socks_grey": 0.0, "socks_red": -0.055}


def placed(x, which=("socks_navy", "socks_grey", "socks_red")):
    return {"drawer": drawer_at(x), **{n: (sock_on(x, YS[n]) if n in which else START[n]) for n in t.SOCKS}}


cases = {
    "start": (START, 0.0),
    "one pair on the open drawer": (placed(OPEN_X, ("socks_navy",)), 0.18),
    "all three on the open drawer": (placed(OPEN_X), 0.54),
    "all three in, drawer closed": (placed(SHUT_X), 0.9),
    "closed with two in (one scraped onto the table)": (
        {**placed(SHUT_X, ("socks_navy", "socks_grey")), "socks_red": st((0.20, -0.02, t.TOP + 0.01))}, 0.36),
    "closed, but a pair on the drawer's front, outside the chest": (
        {**placed(SHUT_X, ("socks_navy", "socks_grey")), "socks_red": sock_on(SHUT_X, -0.055, dx=-0.04)}, 0.54),
    "drawer closed, no socks": ({**START, "drawer": drawer_at(SHUT_X)}, 0.0),
    "drawer half shut with all three": (placed((OPEN_X + SHUT_X) / 2), 0.54),
    "a pair stacked on another (not on the drawer itself)": (
        {**placed(OPEN_X, ("socks_navy", "socks_grey")), "socks_red": sock_on(OPEN_X, 0.0, dz=t.SOCK[2])}, 0.36),
    "a pair left on the chest's top": (
        {**placed(SHUT_X, ("socks_navy", "socks_grey")),
         "socks_red": st((0.32, 0.0, t.TOP + t.INNER_H + t.TOP_T + 0.01))}, 0.36),
    "a pair fell off the table": ({**placed(SHUT_X), "socks_red": st((0.1, -0.3, t.TOP - 0.3))}, 0.0),
}

ok = True
for name, (objs, want) in cases.items():
    got = t.score(objs)
    line = f"{name:56s} {got:.3f}  {t.tally(objs)}"
    if abs(got - want) > 1e-6:
        ok = False
        line += f"  <-- expected {want:.3f}"
    print(line)
assert t.all_done(placed(SHUT_X))
d = t.demo()
print("demo seconds:", round(max(sum(w.duration for w in d[a]) for a in d), 1))
print("ALL OK" if ok else "MISMATCH")
sys.exit(0 if ok else 1)
