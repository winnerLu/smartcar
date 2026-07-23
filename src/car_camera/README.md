# car_camera

罗技 C270 图像采集、八标签停车板检测和四向视觉泊车。

## 节点与话题

| 节点 | 功能 | 主要输出 |
|---|---|---|
| `car_camera_node` | USB 相机采集和已标定内参发布 | `/camera/image_raw`, `/camera/camera_info` |
| `apriltag_detector_node` | 检测 tag36h11 ID 0–7 并联合估算整板位姿 | `/apriltag_detector/tag_pose`, `visible_tag_count`, `visible_tag_ids`, `reprojection_error`, `inlier_count`, TF `camera_optical_frame→parking_board` |
| `board_parker.py` | 四个方向等价的低速停车控制器 | `/cmd_vel_dock`, `/board_parker/parking_complete` |
| `tag_follower.py` | 旧版单标签跟随控制器，仅保留作对比 | `/cmd_vel_dock` |

正常融合链路为：

```text
camera → AprilTag detector → board pose → board_parker
                                                │
                                                ▼
Nav2 /cmd_vel ───────────────┐             /cmd_vel_dock
                             ▼                  │
                          twist_mux ◀───────────┘
                             │
                       /cmd_vel_raw
                             │
                     collision_monitor
                             │
                       /cmd_vel_safe
                             │
                          car_base
```

`/cmd_vel_dock` 的优先级高于 Nav2。`board_parker` 因此默认不激活且
不发布任何速度，只有显式调用服务后才接管控制权。

## 依赖与编译

```bash
sudo apt update
sudo apt install -y \
  ros-humble-camera-info-manager \
  ros-humble-ament-cmake-pytest \
  libapriltag-dev

cd ~/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select car_camera \
  --executor sequential
source install/setup.bash
```

## 第一阶段：只检测，不动车

这是部署后的默认测试方式：

```bash
ros2 launch car_camera board_parking.launch.py \
  video_device:=/dev/camera_c270
```

另开终端检查：

```bash
source ~/robot_ws/install/setup.bash
ros2 topic hz /camera/image_raw
ros2 topic echo /apriltag_detector/tag_pose
ros2 topic echo /apriltag_detector/reprojection_error
ros2 topic echo /apriltag_detector/visible_tag_ids
ros2 topic hz /cmd_vel_dock
```

最后一条应一直收不到消息。把停车板放到相机不同位置并旋转四个方向，
检查所发布的停车板中心是否稳定。RViz 的 Fixed Frame 可设为
`camera_optical_frame`，添加 Image、Pose 和 TF 显示。

## 第二阶段：单独低速测试泊车

先停止 Nav2 和其他速度发布者。若底盘直接订阅 `/cmd_vel`，可执行：

```bash
ros2 launch car_camera board_parking.launch.py \
  cmd_topic:=/cmd_vel \
  parking_enabled:=true \
  target_forward:=0.082 \
  target_left:=0.0 \
  max_linear:=0.05 \
  max_angular:=0.30
```

下视相机的 `z` 是镜头离地高度，不能作为前进距离。节点会先把板位姿
转换到 `base_link`；当前车体几何中心位于驱动轮轴心前方 0.082 m，
因此默认 `target_forward=0.082`。
也可以保持 `parking_enabled:=false`，准备完成后通过服务接管：

```bash
ros2 service call /board_parker/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

立即停车并释放控制权：

```bash
ros2 service call /board_parker/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

## 与 Nav2 的控制权规则

- Nav2 仍发布 `/cmd_vel`，优先级 50。
- 视觉泊车发布 `/cmd_vel_dock`，优先级 100。
- 检测可以始终运行；未激活的 `board_parker` 不发布速度。
- 激活时控制器先发布零速，再等待一帧新的停车板位姿。
- 一个有效 Tag 也能根据唯一 ID 和板内偏移反推出整板位姿，但使用较低
  速度和更长的完成稳定时间。
- 两个及以上 Tag 使用所有角点统一 `solvePnPRansac` 并细化位姿。
- 标签位姿或质量话题超过 `loss_timeout` 未更新时立即发布零速。
- 达标后 `/board_parker/parking_complete` 变为 `true`，并保持零速。
- 上层任务收到完成状态后必须关闭 `board_parker`，让 `twist_mux`
  超时释放泊车输入。
- 禁止在 Nav2 运行时让视觉控制器直接发布 `/cmd_vel`。

## 关键参数

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `target_forward` | `0.082` | m | 停车板中心在 `base_link` 前方的目标位置 |
| `target_left` | `0.0` | m | 停车板中心在 `base_link` 左侧的目标位置 |
| `min_visible_tags` | `1` | 个 | 唯一 ID 的单 Tag 可进行降速泊车 |
| `board_margin` | `0.005` | m | 车体轮廓距离停车板边界的最小余量 |
| `max_linear` | `0.05` | m/s | 多 Tag 最大线速度 |
| `single_tag_max_linear` | `0.025` | m/s | 单 Tag 降级最大线速度 |
| `max_angular` | `0.30` | rad/s | 多 Tag 最大角速度 |
| `position_tolerance` | `0.025` | m | 板中心相对车体目标的径向容差 |
| `edge_tolerance_deg` | `15.0` | ° | 最近停车板边缘方向容差 |
| `min_footprint_overlap` | `0.90` | — | 宣布完成所需的最小车体覆盖率 |
| `max_reprojection_error` | `3.0` | px | 整板位姿最大重投影均方根误差 |
| `loss_timeout` | `0.35` | s | 位姿失效后停车的超时 |
| `stable_frames` | `6` | 帧 | 多 Tag 连续满足容差后宣布完成 |
| `single_tag_stable_frames` | `12` | 帧 | 单 Tag 完成时的额外稳定确认 |
| `allow_reverse` | `true` | — | 允许越过目标后做受限倒车修正 |

相机内参文件为 `config/c270_calibration.yaml`，标定分辨率 640×480。
`tag_size` 是单个 AprilTag 黑色方块实测边长，`board_width` 和
`board_height` 是整块停车板的实测尺寸，单位均为米。

更完整的泊车标定和验收步骤见 [PARKING.md](PARKING.md)。
