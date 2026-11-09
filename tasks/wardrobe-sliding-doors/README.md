# Shelve a sweater behind a wardrobe's sliding doors and close them again

A bimanual YAM task in Isaac Lab Arena. A small wardrobe has two sliding doors in parallel floor tracks: the front
door covers the left half and the rear door the right half, and either can slide across behind or in front of the
other, so only one half can be open at a time. The robot has to put a folded sweater on the left half's shelf:
slide the front door across to open that half, bring the sweater in through the doorway, slide the door back so
both doors are closed, and let go. The task measures working articulated furniture around a placement: choosing
the door that opens the right half, reaching in only through the opening, and leaving the wardrobe closed.

The whole task is one task file, [`wardrobe_sliding_doors.py`](wardrobe_sliding_doors.py), for the `yam_arena`
embodiment from [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (Isaac Sim 6, Isaac Lab
3, Isaac Lab Arena 0.3). The scene is built from boxes only, so nothing is downloaded.

## Run a trial

From this folder, in the Arena environment:

```bash
INSTRUCTION="Put the folded sweater on the shelf in the left half of the wardrobe: slide a door to open that half, bring the sweater in through the doorway, never over a door or a wall, then slide the door back so both doors are closed, and let go."

# Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=wardrobe_sliding_doors.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=wardrobe_sliding_doors.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/demo

# One MP4 per camera, in runs/<policy>/frames/<run>/
inspect-robots video runs/dream/*.json
```

The run prints `task_score` at the end and keeps it in the log.

- **Run limit:** 1,200 control steps (60 s at 20 Hz).
- **Seeds:** none. `-E object_jitter=0` starts every object exactly where the task file puts it, the layout in
  the screenshot. The scripted demo is open-loop and expects that layout.
- **Runtime per trial:** the scripted demo's trial ends at full score after about 940 steps (47 s simulated) and took about 3 minutes of wall time on an L40S, Isaac Sim start included. The Dream-1 trial ran the full 1,200 steps in about 3.5 minutes. Add a few minutes for Isaac Sim's first start on a fresh machine.

## Scene

Positions are in cm, from the midpoint between the two arm bases: x forward, y left, heights above the table
top. The arm bases are at y = +30.5 (left) and y = -30.5 (right).

| Object | Type and material | Size | Position | Mass | |
|---|---|---|---|---|---|
| Side walls (2) | wooden panels | 1 cm thick, 16 cm tall | inner faces at y +13.5 and -13.5, from x 23.2 to the back wall | | fixed |
| Back wall | wooden panel | 1 cm thick, 29 cm wide, 16 cm tall | front face at x 42 | | fixed |
| Divider | wooden panel between the two halves | 1 cm thick, 16 cm tall | y -0.5 to +0.5, x 31 to 42 | | fixed |
| Shelves (2) | white blocks, one in each half | 11 (x) x 13 (y) x 5 cm tall | x 31 to 42; left y +0.5 to +13.5, right y -0.5 to -13.5 | | fixed |
| Floor rails (4) | grey strips forming two tracks | 0.5 cm thick, 1.5 cm tall, 27 cm long | front track centre x 25, rear track centre x 29.2; 0.2 cm clear of each door | | fixed |
| Front door | light wooden panel standing in the front track | 1.2 (x) x 14.5 (y) x 12 cm tall | y -1 to +13.5 (covers the left half) | 200 g | slide across, then back |
| Rear door | darker wooden panel standing in the rear track | 1.2 x 14.5 x 12 cm | y -13.5 to +1 (covers the right half) | 200 g | stays closed |
| Sweater | box, green knitted wool, folded | 8 (x) x 5 (y) x 2.5 cm | lying flat at x 16, y +19 | 120 g | → left shelf |

The doors overlap by 2 cm in the middle when closed. The 3 cm between the two doors' facing sides is the gap
between the two channels of a bypass track.

## Scoring

Scored by `score()` and `success()` in the task file, every control step, from the object poses. Extents are
computed from each box's pose and true size.

- **Hard 0:**
  - at any time in the trial, the sweater passes over a door, a wall or the divider (its footprint overlapping
    theirs, at any height). The wardrobe has no top, so that top-down grippers fit; this rule stands in for the top
    and means the sweater can only reach a shelf through the doorway;
  - the sweater or a door falls off the table (its lowest point more than 5 cm below the table top);
  - a door ends out of its track: tilted more than 10 degrees, its underside more than 5 mm off the table, or its
    centre more than 4 mm off its track's centre line.
- **0.45:** the sweater lying flat (tilt under 10 degrees) and wholly on the left shelf, its underside within 5 mm
  of the shelf top.
- **0.9:** that, and both doors closed: each within 1 cm of its closed position along y.
- **1.0:** 0.9, everything still (under 1 cm/s and 0.5 rad/s), and every fingertip of both grippers at least 3 cm
  from the sweater and the doors, held for 1 s in a row. This ends the trial.

Examples: doing nothing scores 0. Lifting the sweater over the closed doors onto the shelf scores 0. Opening the
right half (sliding the rear door) and shelving the sweater there scores 0. The sweater on the left shelf with the
front door left open scores 0.45.

The trial-level rule needs the history of the trial, so the scorer keeps a record that restarts whenever the
embodiment's episode clock goes back to 0 (every reset).

To score a real trial: watch the trial (or its video) for the sweater passing over a door or a wall; at the end,
check that the sweater lies wholly on the left shelf and measure each door's gap to its closed position with a
tape. "Let go" means both grippers are open and at least 3 cm from the sweater and the doors for a second.

`test_scorer.py` checks the scorer offline (no simulator), with a stand-in for the episode clock, against these and
other made-up trials: `python test_scorer.py`.

## Scripted demo

`demo()` plays an open-loop plan (`yam_scripted` policy). The right arm grips the front door's top edge across its
1.2 cm thickness, 3.25 cm right of the door's centre, with its fingers opened only 1.2 cm wider than the door so the
back finger stays in the gap between the doors, and drags the door across to the right wall. While the right arm
waits parked, the left arm picks up the sweater, carries it round in front of the wardrobe and in through the left
doorway, sets it down on the shelf, opens, narrows its fingers and rises above the doors before backing out. Then
the right arm drags the front door back to the left wall. Each slide drives the fingers 1.5 cm past the wall, so the
door ends against it whatever it slips in the grip. The plan reaches full score after about 940 control
steps.

## Fidelity notes

- The wardrobe is at about a third of real size and has no top, so that top-down grippers fit; the rule that the
  sweater may never pass over a door or a wall stands in for the top.
- The doors are free rigid panels standing in floor tracks, without top guides, rollers or handles; the robot moves
  them by their top edges.
- The sweater is a rigid folded block, since task files have no cloth.
- Friction is the simulator default for every object; it can't be set per material in a task file.
- Grasp, contact and release are approximate. The release check uses the fingertip positions from the arms'
  joint states by forward kinematics (below).

## Dependencies

- yam-isaac-lab-arena at commit `0059d02a1a91bc6834c1d0846022891df0c78730` (2026-11-01), with Isaac Sim 6.1,
  Isaac Lab 3.0 and Isaac Lab Arena 0.3.
- `success()` and `score()` read the embodiment that calls them (found up the call stack): its episode clock, to
  tell trials apart for the trial-level rule, and the arms' joint positions, for the release check by
  `yam_arena.urdf.forward_kinematics`. This depends on the embodiment's private state at the commit above; if the
  state can't be found, scoring raises an error instead of guessing.
