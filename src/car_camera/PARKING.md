# Four-way board parking

The parking target is the physical centre of the eight-tag board. All four
approach directions are valid. The controller aligns to the nearest board edge,
so it never tries to rotate toward a fixed tag ID or a fixed absolute heading.
Approaching the centre from an edge naturally makes the vehicle pass over that
edge's middle tag.

## Control ownership

The detector and controller may stay running during navigation, but the
controller starts **inactive** and publishes no velocity. This is required
because `/cmd_vel_dock` has priority 100 in `twist_mux`, while Nav2
`/cmd_vel` has priority 50. Publishing zero continuously would still block
Nav2.

Start the camera and detector without moving the vehicle:

```bash
ros2 launch car_camera board_parking.launch.py
```

Verify that the controller is inactive and that no dock velocity is being
published:

```bash
ros2 param get /board_parker enabled
ros2 topic hz /cmd_vel_dock
ros2 topic echo /apriltag_detector/tag_pose
```

`ros2 topic hz /cmd_vel_dock` must report that no messages are received. To
explicitly transfer velocity ownership to visual parking:

```bash
ros2 service call /board_parker/set_enabled std_srvs/srv/SetBool "{data: true}"
```

To stop and release ownership back to Nav2:

```bash
ros2 service call /board_parker/set_enabled std_srvs/srv/SetBool "{data: false}"
```

The controller publishes `/board_parker/parking_complete` with transient-local
QoS. It becomes `true` after the pose is within tolerance for the configured
number of consecutive frames. While active and complete, the controller holds
zero velocity; the coordinator must then call `set_enabled: false` to release
the mux input.

Before any driving test, verify the detector output while moving and rotating
the board by hand:

```bash
ros2 topic echo /apriltag_detector/tag_pose
ros2 topic echo /apriltag_detector/visible_tag_ids
ros2 topic echo /apriltag_detector/reprojection_error
ros2 topic echo /apriltag_detector/inlier_count
```

The reported centre must remain at the same physical point when different tags
are visible. Set `board_width`, `board_height`, and `tag_size` to measured print
dimensions. Incorrect print scaling directly produces a biased board centre.

Each ID has a known offset from the board centre. A single valid tag therefore
determines the complete board pose; it is not treated as the parking target
itself. One-tag tracking uses square-tag IPPE, reduced speed, and a longer
completion confirmation. With two or more tags, every visible corner enters
one RANSAC PnP solve followed by LM refinement. Motion stops immediately when
the pose is stale, has fewer than four inlier corners, or exceeds the configured
reprojection-error limit, unless a previously quality-gated board pose is still
inside the bounded odometry-bridge window.

The board centre is intentionally free of tags, so an oblique one-tag approach
can lose its only visible corner tag before another edge tag enters the image.
The controller stores the last confirmed board pose in `odom` and may bridge
that blind interval for at most 6 seconds or 12 cm. Bridge motion is limited to
0.018 m/s and 0.12 rad/s. It automatically returns to vision when a tag
reappears. Odometry alone can never declare parking complete; the final pose
must be visually confirmed.

## Downward camera calibration

The board is on the floor and the camera looks downward. Camera-frame `z` is
therefore camera height, not remaining forward distance. The controller
transforms the detected pose into `base_link` and controls with:

- `base_link.x`: board centre forward/back position;
- `base_link.y`: board centre left/right position.

`base_link` is the drive-axle midpoint. The measured footprint extends 19.7 cm
forward and 3.3 cm backward, so its geometric centre is 8.2 cm ahead of
`base_link`. The calibrated target is consequently:

```text
target_forward = 0.082 m
target_left = 0.000 m
```

The current base-to-optical transform was measured with the chassis geometric
centre over the board centre. Its launch defaults are:

```text
translation = (0.082651, -0.004696, 0.579014) m
quaternion  = (0.706269, -0.707636, 0.017287, 0.011716)  # x y z w
```

Recalibrate these values whenever the pole, camera angle, camera height, or
camera mounting position changes. The controller supports oblique approaches:
it turns toward the board centre, follows a shallow arc, and converges to the
nearest of the board's four equivalent edge headings. At activation it locks
that physical edge direction in `odom` and converts the board centre plus the
8.2 cm axle offset into one fixed drive-axle goal pose. This prevents the
nearest-edge choice from flipping at ±45 degrees. Close to the axle goal, the
singular target-bearing term fades out and the locked heading takes priority.
`allow_reverse=true` permits only a bounded correction after the vehicle has
passed the target; it is not a general reverse-driving mode.

The multi-tag limits are `max_linear=0.05 m/s` and
`max_angular=0.30 rad/s`. A one-tag estimate is deliberately limited to
`0.025 m/s` and `0.18 rad/s`.

For a standalone direct-output test with Nav2 and `twist_mux` both stopped,
override the command topic explicitly:

```bash
ros2 launch car_camera board_parking.launch.py \
  cmd_topic:=/cmd_vel parking_enabled:=true \
  target_forward:=0.082 target_left:=0.0 \
  max_linear:=0.05 max_angular:=0.30
```

Do not use `cmd_topic:=/cmd_vel` while Nav2 is running.

## Acceptance check

Start with detection-only tests at several oblique angles, then run low-speed
tests with two or more visible tags. Test the one-tag fallback last by covering
all but one tag without changing the board pose.

For each approach, confirm that:

- the published board centre stays stable when the set of visible IDs changes;
- a target more than 55 degrees to the side causes rotation before translation;
- loss of the tag or poor reprojection quality produces zero velocity;
- a short loss over the blank board centre enters `mode=odom_hold`, then either
  reacquires vision or stops at the configured time/distance limit;
- the vehicle converges to the nearest board-edge heading rather than a fixed ID;
- `/board_parker/parking_complete` changes to `true` and velocity becomes zero.

Completion requires a fresh, quality-gated pose, radial target error no greater
than 2.5 cm, nearest-edge error no greater than 15 degrees, and at least 90% of
the measured chassis footprint within the board margin. These conditions must
hold for six multi-tag frames or twelve single-tag frames. The relaxed overlap
criterion allows a practical slightly oblique final pose while still preventing
the controller from stopping merely because one tag is visible.
