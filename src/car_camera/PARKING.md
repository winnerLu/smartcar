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

## Required calibration

`target_z` is the board-centre depth at which the vehicle centre is physically
over the board centre. It includes camera mounting offset and cannot be inferred
from the image alone. Measure it on the successful parking setup and pass it at
launch time:

```bash
ros2 launch car_camera board_parking.launch.py target_z:=0.083
```

Tune `target_x` if the camera optical axis is not on the vehicle centreline.
Keep `allow_reverse` false for initial tests. The first standalone tests use
the conservative defaults `max_linear=0.03 m/s` and
`max_angular=0.20 rad/s`. The controller stops immediately when the board pose
is stale and requires eight consecutive in-tolerance frames before declaring
success.

For a standalone direct-output test with Nav2 and `twist_mux` both stopped,
override the command topic explicitly:

```bash
ros2 launch car_camera board_parking.launch.py \
  cmd_topic:=/cmd_vel parking_enabled:=true \
  target_z:=0.083 max_linear:=0.03 max_angular:=0.20
```

Do not use `cmd_topic:=/cmd_vel` while Nav2 is running.

## Acceptance check

Test all four board rotations. For each one, confirm that the vehicle crosses the
middle tag on the approached edge, the chassis centre finishes over the physical
board centre, and `/cmd_vel` becomes zero. Centre error is the primary acceptance
metric; the final heading only needs to be within the configured nearest-edge
tolerance.
