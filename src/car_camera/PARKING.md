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
```

The reported centre must remain at the same physical point when different tags
are visible. Set `board_width`, `board_height`, and `tag_size` to measured print
dimensions. Incorrect print scaling directly produces a biased board centre.

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
camera mounting position changes. Keep `allow_reverse` false for initial tests.
The first standalone tests use the conservative defaults
`max_linear=0.03 m/s` and `max_angular=0.20 rad/s`.

For a standalone direct-output test with Nav2 and `twist_mux` both stopped,
override the command topic explicitly:

```bash
ros2 launch car_camera board_parking.launch.py \
  cmd_topic:=/cmd_vel parking_enabled:=true \
  target_forward:=0.082 target_left:=0.0 \
  max_linear:=0.03 max_angular:=0.20
```

Do not use `cmd_topic:=/cmd_vel` while Nav2 is running.

## Acceptance check

Test all four board rotations. For each one, confirm that the vehicle crosses the
middle tag on the approached edge, the chassis centre finishes over the physical
board centre, and `/cmd_vel` becomes zero. Completion requires at least three
visible tags, a fresh non-jumping pose, the base-frame target within tolerance,
the nearest board edge within 10 degrees, and every measured footprint corner
inside the board with the configured safety margin.
