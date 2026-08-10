# Current end-to-end audit — 2026-07-30

## Verdict

The semantic architecture is now deliberately small:

1. Hydra creates and saves the labeled scene graph during mapping.
2. Runtime semantic actions search that graph first.
3. Free-text labels are matched against the labels and instances in the graph.
4. If an object has no sufficiently good usable graph match, one GroundingDINO
   request is made against the current front RGB-D view.
5. The resulting live observation is converted into a navigable standoff and
   temporarily added to semantic memory.
6. Pick reuses that same pose after navigation, so the fallback remains one
   detection rather than becoming a second close-range guess.
7. Place performs a close-range support-surface check after graph navigation.

There are no task-specific object lists, no hardcoded `blue ball` rule, no
keyframe sweep, and no refusal to save an incomplete graph. Save validation
produces warnings only.

## Saved graph

The current saved artifacts are consistent:

- 209 Hydra place nodes
- 2 rooms
- 9 objects
- object labels: `bed`, `box`, `storage`, and `seating`

The configured generic taxonomy also contains `bottle`, `ball`, and
`plaything`; it does not contain `blue ball` or another task-specific label.
Color is not encoded by the current closed-set semantic segmentation input.

The absence of the ball and cans is plausible with the current Hydra input.
Small objects can be missed by the low-rate, closed-set SegFormer stream or
fail to form a stable 3-D cluster. This is a mapping-quality limitation, not a
corrupt DSG. Hydra's `dynamic_objects` visualization is expected to remain
empty for balls and cans because only `animal` and `human` are configured as
dynamic classes. Static semantic objects appear in the normal graph markers.

## Verified in the running simulation

- The saved graph loaded successfully and `bed` resolved to Hydra node `O18`
  with confidence `1.0`.
- The Hydra target was projected from an unknown 2-D map cell to a connected,
  robot-clear Nav2 cell at approximately `(-7.526, 1.878)`.
- An unknown `blue ball` caused exactly one current-camera GroundingDINO
  request, produced a 3-D observation, and resolved to a live navigation
  standoff.
- Navigation reached the live ball standoff at about 0.52 m from the measured
  ball position.
- The same live pose was passed to `PickObject`; no second GroundingDINO
  request ran in the clean attempt.
- A focused call through the rebuilt production manipulator service completed
  the full pickup sequence successfully: pre-grasp, low grasp, gripper close,
  and lift.
- The generated full BT is structurally correct: ball route, `PickObject`,
  graph-native bed route, then `PlaceObject`. The separately generated
  "place the object on the bed" BT also contained only the bed route and
  `PlaceObject`. Generation therefore worked in the current no-credit setup
  without requiring a graph-class prompt dump or label-specific rewriting.
- VNC connected successfully, but its RViz view stayed visually static during
  the final diagnostic. No visual motion is claimed.

## Remaining runtime issue

### Final placement still needs one clean confirmation

After the successful focused pickup, the command `place the object on the bed`
resolved `bed` to Hydra node `O18`, compiled a Nav2 goal at approximately
`(-7.526, 1.878)`, and traversed the full route from the ball area. AMCL showed
the robot at `(-7.635, 1.725)` after the abort: only 0.19 m from the goal.
The abort was the overly strict final yaw/position shuffle, not a stuck base or
semantic lookup failure.

With a runtime 0.20 m / 0.80 rad terminal tolerance, the same bed route
immediately completed and `PlaceObject` ran. It detected the bed at 0.72 m
camera depth, recovered a support pose 0.75 m from the base, and produced a
valid arm plan. Execution timed out because 0.75 m is beyond the loaded arm's
reliable reach. The final source configuration therefore keeps 0.10 m
translation accuracy, relaxes only terminal yaw to 0.80 rad, and refuses
placement poses beyond 0.70 m. This combination has not yet had a clean
end-to-end confirmation.

The most useful next run is one clean, uninterrupted command:

```text
pick up the blue ball and place it on the bed
```

If it aborts after pickup, capture the `controller_server`, `PlaceObject`,
`manipulator_service`, and `bt_navigator` lines beginning about ten seconds
before `Goal failed`. In particular, report the detected support distance and
whether Nav2 reports `Reached the goal`.

Successful physical placement on the bed is therefore still unverified.

## Other cleanup

GroundingDINO now fails closed if its real model cannot load; mock detections
are available only when explicitly requested. The live fallback uses a
multithreaded executor so RGB-D, TF, and detector callbacks do not deadlock.
The live RGB-D freshness allowance is five seconds to tolerate the CPU-only
simulation bridge, while still using exactly one current front-camera frame.

The saved-map projection now preserves a 0.45–0.85 m manipulation standoff for
live fallback observations without applying that point-object constraint to
large graph-native furniture. The manipulator uses a 0.22 m model-reachable
tool-height floor, and the simulated arm controller has research-appropriate
settling tolerances.

The Foxglove bridge now excludes Hydra's private mesh/DSG message types, which
are installed only in the companion container, while leaving Hydra's standard
visualizer marker topics available. This prevents the repeated missing-schema
log flood on future launches.
