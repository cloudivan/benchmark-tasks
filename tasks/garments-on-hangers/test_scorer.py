"""Offline checks of the scorer against made-up outcomes (no simulator). Run: python test_scorer.py"""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))
import garments_on_hangers as t  # noqa: E402

ID = (0.0, 0.0, 0.0, 1.0)


def yawq(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def st(pos, q=ID):
    return SimpleNamespace(pos=pos, quat_xyzw=q, resting=lambda: True)


def on_bar(name, hanger, dx=0.0, dy=0.0, q=ID):
    s = t.ITEMS[name]
    return st((t.HANGER_X + dx, t.HANGERS[hanger][0] + dy, t.TOP + t.BAR_TOP_H + s.size[2] / 2), q)


def on_shelf(name, dx=0.0):
    s = t.ITEMS[name]
    return st((t.SHELF_CENTRE[0] + dx, t.SHELF_CENTRE[1], t.SHELF_TOP + s.size[2] / 2))


start = {n: st(b.pos) for n, b in t.ITEMS.items()}
done = {"trousers": on_bar("trousers", "trouser_hanger"), "silk_blouse": on_bar("silk_blouse", "padded_hanger"),
        "wool_sweater": on_shelf("wool_sweater")}

cases = {
    "start": (start, 0.0),
    "all right": (done, 0.9),
    "trousers only": ({**start, "trousers": done["trousers"]}, 0.3),
    "garments swapped between the hangers": ({**start, "trousers": on_bar("trousers", "padded_hanger"),
                                               "silk_blouse": on_bar("silk_blouse", "trouser_hanger")}, 0.0),
    "sweater hung, others right": ({**done, "wool_sweater": on_bar("wool_sweater", "trouser_hanger", dy=-0.02)},
                                   0.3),
    "blouse 2 cm off the bar (front-back)": ({**start, "silk_blouse": on_bar("silk_blouse", "padded_hanger",
                                                                             dx=0.02)}, 0.0),
    "trousers along the bar, not across": ({**start, "trousers": on_bar("trousers", "trouser_hanger",
                                                                       q=yawq(90))}, 0.0),
    "trousers over the end of the bar (hanging past it)": ({**start, "trousers": on_bar("trousers",
                                                                                       "trouser_hanger",
                                                                                       dy=0.04)}, 0.0),
    "sweater half off the shelf": ({**start, "wool_sweater": on_shelf("wool_sweater", dx=0.03)}, 0.0),
    "all right but blouse fell off the table": ({**done, "silk_blouse": st((0.2, -0.1, t.TOP - 0.3))}, 0.0),
    "trousers on shelf, blouse right": ({**start, "trousers": st((0.22, 0.0, t.SHELF_TOP + 0.01)),
                                         "silk_blouse": done["silk_blouse"]}, None),
}

ok = True
for name, (objs, want) in cases.items():
    got = t.score(objs)
    line = f"{name:58s} {got:.3f}  {t.tally(objs)['where']}"
    if want is not None and abs(got - want) > 1e-6:
        ok = False
        line += f"  <-- expected {want:.3f}"
    print(line)
assert t.all_done(done)
d = t.demo()
print("demo seconds:", round(max(sum(w.duration for w in d[a]) for a in d), 1))
print("ALL OK" if ok else "MISMATCH")
sys.exit(0 if ok else 1)
