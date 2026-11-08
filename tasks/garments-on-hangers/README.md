# Put trousers and a silk blouse on their own hangers, and the knitted sweater on the shelf

A bimanual YAM task in Isaac Lab Arena. Two open-ended hangers hang from a clothes rail: a wooden trouser hanger
and a pink padded hanger for delicates. A shelf stands in front of the rail. Three folded garments lie on the
table: trousers, a silk blouse and a wool sweater. The robot has to lay the trousers across the trouser hanger's
bar and the blouse across the padded hanger's bar, each with its middle on the bar so it doesn't slide off, lay
the sweater flat on the shelf (knits stretch out of shape on a hanger), and let go. Swapping the two garments
between the hangers scores 0. The task measures matching each garment to its storage and a balanced placement on
a narrow bar.

The whole task is one task file, [`garments_on_hangers.py`](garments_on_hangers.py), for the `yam_arena`
embodiment from [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (Isaac Sim 6, Isaac Lab
3, Isaac Lab Arena 0.3). The scene is built from boxes only, so nothing is downloaded.

## Run a trial

From this folder, in the Arena environment:

```bash
INSTRUCTION="Put the clean laundry away: lay the trousers across the bar of the wooden trouser hanger and the silk blouse across the bar of the pink padded hanger, each with its middle on the bar, and lay the wool sweater flat on the shelf, because knits stretch out of shape on a hanger. Then let go of each one."

# Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=garments_on_hangers.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=garments_on_hangers.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 1200 --store-frames --log-dir runs/demo

# One MP4 per camera, in runs/<policy>/frames/<run>/
inspect-robots video runs/dream/*.json
```

The run prints `task_score` at the end and keeps it in the log.

- **Run limit:** 1,200 control steps (60 s at 20 Hz).
- **Seeds:** none. `-E object_jitter=0` starts every object exactly where the task file puts it, the layout in
  the screenshot. The scripted demo is open-loop and expects that layout.
- **Runtime per trial:** the scripted demo's trial ends at full score after about 615 steps (31 s simulated) and took about 2 minutes of wall time on an L40S, Isaac Sim start included. The Dream-1 trial ran the full 1,200 steps in about 4 minutes. Add a few minutes for Isaac Sim's first start on a fresh machine.

## Scene

Positions are in cm, from the midpoint between the two arm bases: x forward, y left, heights above the table
top. The arm bases are at y = +30.5 (left) and y = -30.5 (right).

| Object | Type and material | Size | Position | Mass | |
|---|---|---|---|---|---|
| Clothes rail | grey steel bar on two posts | rail 1.5 x 48 x 1.5 cm, posts 1.5 x 1.5 x 22 cm | rail along y at x 30, 22 cm up; posts at y +23.5 and -23.5 | | fixed |
| Trouser hanger | varnished wooden bar, open-ended, its hook stem (0.6 x 0.6 cm) at its left end rising to the rail | bar 2.5 (x) x 10 (y) x 1.2 cm | bar centred x 30, y +10, top face 10 cm up | | fixed |
| Padded hanger | pink satin-padded bar, open-ended, stem at its right end | bar 3 x 10 x 2.2 cm | bar centred x 30, y -10, top face 10 cm up | | fixed |
| Shelf | white block | 12 x 12 x 4 cm tall | x 22, y 0 | | fixed |
| Trousers | box, navy cotton, folded | 14 (x) x 4.5 (y) x 2.0 cm | x 17, y +10 | 100 g | → across the trouser hanger's bar |
| Silk blouse | box, ivory silk, folded | 12 x 4.5 x 1.6 cm | x 17, y -10 | 40 g | → across the padded hanger's bar |
| Wool sweater | box, grey knitted wool, folded | 8 x 5 x 2.5 cm | x 17, y +20 | 120 g | → flat on the shelf |

The garments lie flat with their long side along x, square to the table edges. Each hanger's bar is held only at
its outer end, so a garment can be laid over it from above, as on an open-ended trouser hanger.

## Scoring

Scored by `score()` and `success()` in the task file, every control step, from the object poses. Extents are
computed from each box's pose and true size.

- **Hard 0:** any garment ends with its lowest point more than 5 cm below the table top (fell off the table).
- **Otherwise:** `0.9 x max(0, C/3 - W/3)`
  - `C`: garments at their right place; `W`: garments at a wrong place (the other hanger, or the shelf).
  - On a hanger: lying across its bar (long axis within 15 degrees of x, tilt under 10 degrees), underside within
    5 mm of the bar's top face, centre over the bar (within its 2.5 or 3 cm front-back width), and the garment's
    whole width within the bar's 10 cm length.
  - On the shelf: tilt under 10 degrees, underside within 5 mm of the shelf top, and wholly over it.
  - Right places: trousers on the trouser hanger, blouse on the padded hanger, sweater on the shelf.
- **1.0:** all three at their right places, all still (under 1 cm/s and 0.5 rad/s), and every fingertip of both
  grippers at least 3 cm from every garment, held for 1 s in a row. This ends the trial.

Examples: doing nothing scores 0. One garment right scores 0.3. Swapping the trousers and the blouse between the
hangers scores 0. Both hung right but the sweater hung on a hanger too scores 0.3. All three right but still
holding one scores 0.9.

To score a real trial: check that each hung garment lies across its bar with the bar under its middle (measure
from the garment's ends with a tape), and that the sweater lies wholly on the shelf. "Let go" means both grippers
are open and at least 3 cm from every garment for a second.

`test_scorer.py` checks the scorer offline (no simulator) against these and other made-up outcomes:
`python test_scorer.py`.

## Scripted demo

`demo()` plays an open-loop plan (`yam_scripted` policy). The left arm lays the trousers across the trouser
hanger's bar while the right arm lays the silk blouse across the padded hanger's bar; then the left arm lays the
sweater flat on the shelf. Each grasp comes down with the fingers fully open, waits 1.5 s for the arm to settle,
and closes slowly across the garment's width to a bounded squeeze. A garment going onto a bar is held 4 cm behind
its middle, so the fingers stay clear of the bar while its middle is set down on it from 3 mm above. The plan
reaches full score after about 615 control steps.

## Fidelity notes

- Garments are rigid folded blocks at about a third of real size, so that the 9.4 cm gripper can hold one across
  its width. Task files have no cloth, so a garment lies across a bar the way folded trousers do, but can't drape,
  crease or be threaded into a fold.
- The hangers are fixed rigid bars, with their hooks fixed to the rail; a real hanger would swing a little.
- The care rules (trousers on a trouser hanger, silk on a padded hanger, knitwear folded rather than hung) are
  common laundry practice and are stated in the instruction.
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
