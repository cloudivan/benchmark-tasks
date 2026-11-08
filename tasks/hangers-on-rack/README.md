# Hang a coat, a shirt and a child's hanger on the rack arms each one fits

A bimanual YAM task in Isaac Lab Arena. A laundry pickup rack has three pairs of arms, spaced for a coat hanger, a
shirt hanger and a child's hanger. Three hangers lie on the table in front of it. The robot has to hang each one on
the pair it fits: its bar across both cradles of that pair, with neither end sticking out more than 1.5 cm past
the arms, and let go. Each hanger fits exactly one pair, by length: a shorter hanger falls between a wider pair's
arms, and a longer one overhangs a narrower pair by 2.25 cm or more. The task measures judging an object's size
against a fixture and a precise two-point placement.

The whole task is one task file, [`hangers_on_rack.py`](hangers_on_rack.py), for the `yam_arena` embodiment from
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (Isaac Sim 6, Isaac Lab 3, Isaac Lab
Arena 0.3). The scene is built from boxes only, so nothing is downloaded.

## Run a trial

From this folder, in the Arena environment:

```bash
INSTRUCTION="Hang each hanger on the pair of rack arms it fits: lay its bar across both cradles of that pair, pointing away from you, with neither end sticking out more than 1.5 cm past the arms, then let go of it."

# Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hangers_on_rack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hangers_on_rack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/demo

# One MP4 per camera, in runs/<policy>/frames/<run>/
inspect-robots video runs/dream/*.json
```

The run prints `task_score` at the end and keeps it in the log.

- **Run limit:** 1,200 control steps (60 s at 20 Hz).
- **Seeds:** none. `-E object_jitter=0` starts every object exactly where the task file puts it, the layout in
  the screenshot. The scripted demo is open-loop and expects that layout.
- **Runtime per trial:** the scripted demo's trial ends at full score after about 580 steps (29 s simulated) and took about 2 minutes of wall time on an L40S, Isaac Sim start included. The Dream-1 trial ran the full 1,200 steps in about 4 minutes. Add a few minutes for Isaac Sim's first start on a fresh machine.

## Scene

Positions are in cm, from the midpoint between the two arm bases: x forward, y left, heights above the table
top. The arm bases are at y = +30.5 (left) and y = -30.5 (right).

| Object | Type and material | Size | Position | Mass | |
|---|---|---|---|---|---|
| Rack arm (6) | grey post with a cradle: two lips 1.5 x 0.5 x 1.2 cm, left and right of the post's top, 2 cm apart | post 1.5 (x) x 3 (y) x 10 cm tall | see the pairs below | | fixed |
| Narrow pair | two arms, one behind the other | inner gap 4.5 cm, outer span 7.5 cm | y +20; posts at x 27 and 33 | | fixed |
| Middle pair | two arms | inner gap 8.5 cm, outer span 11.5 cm | y +6; posts at x 25 and 35 | | fixed |
| Wide pair | two arms | inner gap 12.5 cm, outer span 15.5 cm | y -14; posts at x 23 and 37 | | fixed |
| Coat hanger | box, dark brown wood: the hanger's shoulder bar, standing on its narrow edge | 16 (x) x 1.6 (y) x 3.0 cm tall | x 16, y -10 | 150 g | → wide pair |
| Shirt hanger | box, white plastic, the same way up | 12 x 1.4 x 2.6 cm | x 16, y +16 | 60 g | → middle pair |
| Child's hanger | box, pink plastic, the same way up | 8 x 1.2 x 2.2 cm | x 16, y +4 | 30 g | → narrow pair |

The hangers lie with their long side along x, square to the table edges. Their order on the table (coat, child's,
shirt from right to left) differs from the pairs' order (wide, middle, narrow), so position doesn't tell where a
hanger goes.

Why each hanger fits one pair only (end overhang = (bar length - outer span) / 2):

| | narrow (gap 4.5, span 7.5) | middle (gap 8.5, span 11.5) | wide (gap 12.5, span 15.5) |
|---|---|---|---|
| coat, 16 cm | overhangs 4.25 cm | overhangs 2.25 cm | **fits** (0.25 cm) |
| shirt, 12 cm | overhangs 2.25 cm | **fits** (0.25 cm) | falls through |
| child's, 8 cm | **fits** (0.25 cm) | falls through | falls through |

## Scoring

Scored by `score()` and `success()` in the task file, every control step, from the object poses. Extents are
computed from each box's pose and true size.

- **Hard 0:** any hanger ends with its lowest point more than 5 cm below the table top (fell off the table).
- **Otherwise:** `0.9 x H/3`, `H` = hangers hung on the pair they fit. A hanger is hung on a pair when:
  - its long axis is within 5 degrees of x, and level within 5 degrees;
  - its centre is within 1 cm of the pair's centre line (y) and of the pair's middle (x 30);
  - its underside is within 4 mm of the cradle floors, 10 cm above the table (a bar resting on the lips doesn't
    count);
  - each end overhangs the pair's arms by no more than 1.5 cm.
- **1.0:** all three hung on the pairs they fit, all still (under 1 cm/s and 0.5 rad/s), and every fingertip of
  both grippers at least 3 cm from every hanger, held for 1 s in a row. This ends the trial.

Examples: doing nothing scores 0. One hanger hung scores 0.3, two score 0.6. The coat hanger on the middle pair
counts for nothing (it overhangs by 2.25 cm). All three hung but still holding one scores 0.9.

To score a real trial: for each hanger on the rack, check with a tape that it lies in both cradles of one pair and
measure how far each end sticks out past the arms. "Let go" means both grippers are open and at least 3 cm from
every hanger for a second.

`test_scorer.py` checks the scorer offline (no simulator) against these and other made-up outcomes:
`python test_scorer.py`.

## Scripted demo

`demo()` plays an open-loop plan (`yam_scripted` policy). The right arm hangs the coat hanger on the wide pair
while the left arm hangs the child's hanger on the narrow pair; then the left arm hangs the shirt hanger on the
middle pair. Each grasp comes down over the bar's middle with the fingers fully open, waits 1.5 s for the arm to
settle, closes slowly across the bar's thickness to a bounded squeeze, lifts the bar over the cradle lips, lowers
it to 3 mm above the cradle floors and opens. Pick and place targets include small corrections measured in
simulation. The plan reaches full score after about 580 control steps.

## Fidelity notes

- Each hanger is represented by its shoulder bar, a rigid flat box at about a third of real size (a coat hanger
  is about 45 cm wide), so that the 9.4 cm gripper can hold it across its thickness. Task files have no cloth and
  no linked parts, so the hook and the garment hanging from it are not modelled; the bar rests in the two cradles
  of a pair the way a hanger rests on two rack arms.
- The rack is fixed, like a rack standing on the floor.
- Friction is the simulator default for every object; it can't be set per material in a task file.
- Grasp, contact and release are approximate. The release check uses the fingertip positions from the arms'
  joint states by forward kinematics (below).

## Dependencies

- yam-isaac-lab-arena at commit `0059d02a1a91bc6834c1d0846022891df0c78730` (2026-11-01), with Isaac Sim 6.1,
  Isaac Lab 3.0 and Isaac Lab Arena 0.3.
- `success()` checks the release with the arms' fingertip positions. The task-file hooks receive only object
  states, so it reads the joint positions of the `yam_arena` embodiment that calls it (found up the call stack)
  and runs `yam_arena.urdf.forward_kinematics`. This depends on the embodiment's private state at the commit
  above; if the state can't be found, scoring raises an error instead of guessing.
