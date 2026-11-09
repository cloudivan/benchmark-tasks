# Pack three pairs of socks flat into a drawer so that it closes

A bimanual YAM task in Isaac Lab Arena. A low chest's drawer is pulled 8 cm out, and three rolled pairs of socks lie
on the table. The robot has to put the socks in the drawer and push it shut, then let go. Only the pulled-out part of
the drawer can be reached, and there are 2.6 cm between the drawer and the chest's top: room for one layer of socks.
A pair stacked on another or stood on end is taller than that gap, catches on the chest's front edge and is pushed
off as the drawer closes, as in an overfull drawer. The task measures packing to a clearance the robot has to judge
from the furniture, followed by a controlled push that carries the load.

The whole task is one task file, [`sock_drawer.py`](sock_drawer.py), for the `yam_arena` embodiment from
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (Isaac Sim 6, Isaac Lab 3, Isaac Lab
Arena 0.3). The scene is built from boxes only, so nothing is downloaded.

## Run a trial

From this folder, in the Arena environment:

```bash
INSTRUCTION="Put the three pairs of socks in the drawer and close it, then let go."

# Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sock_drawer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir runs/dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sock_drawer.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1600 --store-frames --log-dir runs/demo

# One MP4 per camera, in runs/<policy>/frames/<run>/
inspect-robots video runs/dream/*.json
```

The run prints `task_score` at the end and keeps it in the log.

- **Run limit:** 1,600 control steps (80 s at 20 Hz): three pick-and-places and a push.
- **Seeds:** none. `-E object_jitter=0` starts every object exactly where the task file puts it, the layout in
  the screenshot. The scripted demo is open-loop and expects that layout.
- **Runtime per trial:** the scripted demo's trial ends at full score after about 1,085 steps (54 s simulated) and took about 3 minutes of wall time on an L40S, Isaac Sim start included. The Dream-1 trial ran the full 1,600 steps in about 4 minutes. Add a few minutes for Isaac Sim's first start on a fresh machine.

## Scene

Positions are in cm, from the midpoint between the two arm bases: x forward, y left, heights above the table
top. The arm bases are at y = +30.5 (left) and y = -30.5 (right).

| Object | Type and material | Size | Position | Mass | |
|---|---|---|---|---|---|
| Chest sides (2) | wooden panels | 13 (x) x 1 (y) x 4.6 cm tall | inner faces at y +8.3 and -8.3, from x 26 to 39 | | fixed |
| Chest back | wooden panel | 1 (x) x 16.6 (y) x 4.6 cm tall | front face at x 38 (12 cm inside the chest) | | fixed |
| Chest top | wooden panel on the sides and back | 13 (x) x 18.6 (y) x 1.2 cm | x 26 to 39; underside 4.6 cm above the table | | fixed |
| Drawer | box, light wood: a solid block standing for the drawer's floor and front | 15 (x) x 16 (y) x 2 cm | front face at x 15 (8 cm out), back face at x 30, y -8 to +8 | 200 g | → push home: back face at the back wall, front face at x 23 |
| Navy socks | box, a rolled pair of cotton socks | 4 (x) x 3 (y) x 2 cm | x 15, y +17 | 30 g | → on the drawer |
| Grey socks | the same | 4 x 3 x 2 cm | x 21, y +17 | 30 g | → on the drawer |
| Red socks | the same | 4 x 3 x 2 cm | x 17, y -17 | 30 g | → on the drawer |

The socks lie flat with their long side along x. The drawer's pulled-out part (x 15 to 26, 11 x 16 cm) has room
for the three pairs side by side in one layer. Closed, the drawer's top is 2.6 cm below the chest's top: one layer
(2 cm) passes, two layers or a pair on end (4 cm) don't. The drawer is 3 cm longer than the chest is deep: closed,
its front 3 cm stand out of the chest like a front panel, so a pair left there is not in the drawer.

## Scoring

Scored by `score()` and `success()` in the task file, every control step, from the object poses. Extents are
computed from each box's pose and true size.

- **Hard 0:** the drawer or a pair of socks falls off the table (its lowest point more than 5 cm below the table top).
- **Otherwise:** `0.9 x (0.2 N + 0.4 C)`
  - `N`: pairs resting on the drawer: underside within 5 mm of the drawer's top and wholly over it, wherever the
    drawer is. A pair on another pair, on the chest's top or on the table doesn't count.
  - `C`: 1 if the drawer is closed (pushed home: its back face within 5 mm of the back wall, tilted under
    10 degrees) with all three pairs on it and wholly inside the chest (behind its front at x 26), else 0.
- **1.0:** all three pairs on the closed drawer, inside the chest, everything still (under 1 cm/s and 0.5 rad/s), and every fingertip
  of both grippers at least 3 cm from the drawer and the socks, held for 1 s in a row. This ends the trial.

Examples: doing nothing scores 0. Closing the empty drawer scores 0. One pair on the open drawer scores 0.18, all
three 0.54. Closing it with only two left on it (the third scraped off at the front edge) scores 0.36; with the third
left on the drawer's front, outside the chest, 0.54. All three in the closed drawer but still touching it scores 0.9.

To score a real trial: look into the chest from the side or lift its top; count the pairs lying on the drawer and
inside the chest, and check that the drawer is pushed home against the back wall. "Let go" means both grippers are open and at least 3 cm from
the drawer and the socks for a second.

`test_scorer.py` checks the scorer offline (no simulator) against these and other made-up outcomes:
`python test_scorer.py`.

## Scripted demo

`demo()` plays an open-loop plan (`yam_scripted` policy), one arm over the drawer at a time. The left arm lays the
grey pair in the middle of the drawer's pulled-out part, 6 cm in from its front, and the navy pair 5.5 cm to its
left, then parks. The right arm lays the red pair 5.5 cm to the right of the grey one, then comes down in front of
the drawer with its fingers closed and pushes the drawer's front face slowly, 8 cm, until the drawer meets the back
wall (the plan pushes 1 cm further; the wall stops the drawer), and backs off. Each grasp comes down with the fingers fully open, waits for the arm to settle and closes
slowly across the pair's 3 cm width; each release opens the fingers only 8 mm wider than the pair, so they don't push
its neighbours. The plan reaches full score
after about 1,085 control steps.

## Fidelity notes

- The chest is at about a third of real size and fixed, like a chest standing against a wall.
- The drawer is one solid block standing for the drawer's floor and front: a task file can't join boxes, so it has
  no sides, and the socks ride on its top. Its front 3 cm stand out of the closed chest, as a drawer's front panel
  does. It slides on the table between the chest's sides.
- Rolled socks are rigid blocks, not soft rolls: a stacked pair can't squash down to fit under the chest's top.
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
