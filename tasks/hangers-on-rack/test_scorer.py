"""Offline checks of the scorer against made-up outcomes (no simulator). Run: python test_scorer.py"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))
import hangers_on_rack as t  # noqa: E402

ID = (0.0, 0.0, 0.0, 1.0)


def yawq(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def st(pos, q=ID):
    return SimpleNamespace(pos=pos, quat_xyzw=q, resting=lambda: True)


def on_pair(name, pair, dy=0.0, q=ID):
    s = t.BARS[name]
    return st((t.RACK_X, t.PAIRS[pair][0] + dy, t.CRADLE_TOP + s.size[2] / 2), q)


start = {n: st(b.pos) for n, b in t.BARS.items()}
right = {n: on_pair(n, t.FITS[n]) for n in t.BARS}

cases = {
    "start": (start, 0.0),
    "all on their own pairs": (right, 0.9),
    "coat only": ({**start, "coat_hanger": right["coat_hanger"]}, 0.3),
    "coat on middle pair (overhangs 2.25 cm)": ({**start, "coat_hanger": on_pair("coat_hanger", "middle")}, 0.0),
    "shirt on narrow pair (overhangs 2.25 cm)": ({**start, "shirt_hanger": on_pair("shirt_hanger", "narrow")}, 0.0),
    "child bar resting at wide-pair height (impossible: falls through; scorer still says no)":
        ({**start, "child_hanger": on_pair("child_hanger", "wide")}, 0.0),
    "coat 2 cm off centre": ({**start, "coat_hanger": on_pair("coat_hanger", "wide", dy=0.02)}, 0.0),
    "coat skewed 10 deg": ({**start, "coat_hanger": on_pair("coat_hanger", "wide", q=yawq(10))}, 0.0),
    "coat resting on top of the lips (1.2 cm high)":
        ({**start, "coat_hanger": st((t.RACK_X, -0.14, t.CRADLE_TOP + 0.012 + 0.008))}, 0.0),
    "all hung but shirt fell off table":
        ({**right, "shirt_hanger": st((0.1, 0.1, t.TOP - 0.3))}, 0.0),
    "two hung": ({**right, "child_hanger": start["child_hanger"]}, 0.6),
}

ok = True
for name, (objs, want) in cases.items():
    got = t.score(objs)
    line = f"{name:88s} {got:.3f}  {t.tally(objs)['where']}"
    if abs(got - want) > 1e-6:
        ok = False
        line += f"  <-- expected {want:.3f}"
    print(line)
assert t.all_hung(right)
d = t.demo()
print("demo seconds:", round(sum(w.duration for w in d["left"]), 1))
print("ALL OK" if ok else "MISMATCH")
sys.exit(0 if ok else 1)
