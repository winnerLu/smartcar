# car_camera — 摄像头采集 + AprilTag 检测 + 视觉伺服泊车

## 节点概览

| 节点 | 语言 | 功能 |
|------|------|------|
| `car_camera_node` | C++ | USB 摄像头采集,发布 `/camera/image_raw` + `/camera/camera_info` |
| `apriltag_detector_node` | C++ | AprilTag `tag36h11` 检测,发布 `/tag_pose` + TF `camera_link→tag_0` |
| `tag_follower.py` | Python | 视觉伺服 P 控制器,发布 `/cmd_vel` 驱动小车自动泊入标签前方 |

## 数据流

```
┌──────────────┐     ┌──────────────────────┐     ┌──────────────┐
│ 摄像头/USB    │────▶│ car_camera_node       │────▶│ /image_raw   │
└──────────────┘     └──────────────────────┘     │ /camera_info │
                                                   └──────┬───────┘
                                                          │
                                                          ▼
┌──────────────┐     ┌──────────────────────┐     ┌──────────────┐
│ /tag_pose    │◀────│ apriltag_detector_node│◀────│              │
│ TF: tag_0    │     └──────────────────────┘     └──────────────┘
└──────┬───────┘
       │
       ▼
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ tag_follower │────▶│ /cmd_vel     │────▶│ car_base →   │
│ (P 控制器)    │     │ (Twist)      │     │ 串口 → 电机   │
└──────────────┘     └──────────────┘     └──────────────┘
```

## 快速开始

### 1. 安装依赖

```bash
sudo apt update
sudo apt install -y ros-humble-camera-info-manager libapriltag-dev python3-pip
pip install pupil-apriltags
```

### 2. 编译

```bash
cd ~/rviz_ros2/smartcar
source /opt/ros/humble/setup.bash
colcon build --packages-select car_camera --symlink-install
source install/setup.bash
```

### 3. 电脑摄像头测试（仅检测）

```bash
ros2 launch car_camera camera_apriltag.launch.py \
    video_device:=/dev/video0 tag_size:=0.120
```

### 4. 电脑摄像头测试（检测 + 模拟控制）

```bash
ros2 launch car_camera camera_apriltag.launch.py \
    video_device:=/dev/video0 tag_size:=0.120 \
    enable_follower:=true
```

### 5. RViz 可视化

```bash
# 新终端
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2
```

RViz 配置:

- **Fixed Frame** → `camera_link`
- **Add → Image** → Topic: `/camera/image_raw`
- **Add → TF** → 查看 `tag_0` 坐标系
- **Add → Pose** → Topic: `/apriltag_detector/tag_pose`

### 6. 查看实时数据

```bash
# 标签位姿
ros2 topic echo /apriltag_detector/tag_pose

# 小车控制指令
ros2 topic echo /cmd_vel

# 图像帧率
ros2 topic hz /camera/image_raw
```

## 日志解读

```
[tag_follower] tag(z=0.171, x=0.033, y=-0.005)
                err(z=0.121, x=0.033, y=-0.005)
                cmd(v=0.080, w=-0.066)
```

| 字段 | 含义 | 示例值解读 |
|------|------|------------|
| `tag.z` | 标签距相机深度 | 0.171m — 标签在 17cm 处 |
| `tag.x` | 标签水平偏移（右正左负） | 0.033m — 偏右 3.3cm |
| `tag.y` | 标签垂直偏移（下正上负） | -0.005m — 略高 |
| `err.z` | 深度误差 `tag.z - target_z` | 0.121m — 还差 12cm |
| `err.x` | 横向误差 | 偏右 3.3cm |
| `cmd.v` | 线速度指令 `linear.x` | 0.080 — 全速前进 |
| `cmd.w` | 角速度指令 `angular.z` | -0.066 — 右转修正 |

## 启动参数

### camera_apriltag.launch.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `video_device` | `/dev/camera_c270` | 摄像头设备路径 |
| `camera_name` | `webcam` | 相机名称（须与标定文件一致） |
| `camera_info_url` | `package://car_camera/config/webcam_calibration.yaml` | 标定文件 URL |
| `tag_size` | `0.120` | AprilTag 黑色方块实测边长（米） |
| `enable_follower` | `false` | 是否启动视觉伺服控制 |

### tag_follower (Python 节点)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_z` | `0.05` | 目标距离（米），到达后停车 |
| `kp_linear` | `0.8` | 深度 P 增益 |
| `kp_angular` | `2.0` | 横向 P 增益 |
| `max_linear` | `0.08` | 最大线速度（m/s） |
| `max_angular` | `0.5` | 最大角速度（rad/s） |
| `dead_zone_x` | `0.003` | 横向死区（米） |
| `dead_zone_y` | `0.003` | 垂直死区（米） |
| `dead_zone_z` | `0.005` | 深度死区（米） |
| `loss_timeout` | `0.5` | 标签丢失超时（秒），超时自动停车 |

### apriltag_detector (C++ 节点)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tag_id` | `0` | 追踪的标签 ID，`-1` 表示检测所有 ID |
| `quad_decimate` | `1.0` | 降采样系数，越小越灵敏（1.0=不降采样） |
| `tag_family` | `tag36h11` | AprilTag 家族 |

## 调参指南

### 灵敏度不够（检测不到标签）

- 降低 `quad_decimate`（当前已是最低 1.0）
- 增加光照或缩小标签与摄像头距离
- 确认标签平整无反光

### 小车动作太猛

```bash
ros2 launch car_camera camera_apriltag.launch.py \
    enable_follower:=true max_linear:=0.05 max_angular:=0.3
```

### 小车反应太慢

```bash
ros2 launch car_camera camera_apriltag.launch.py \
    enable_follower:=true kp_linear:=1.2 kp_angular:=3.0
```

### 停车偏差大

- 先做**相机内参标定**（影响位姿精度）
- 收紧死区 `dead_zone_x:=0.002 dead_zone_z:=0.003`

## 上车部署

### 1. 相机标定

```bash
ros2 run camera_calibration cameracalibrator \
    --size 9x6 --square 0.025 \
    image:=/camera/image_raw camera:=/camera
```

标定完成后得到 `ost.yaml`，放入 `config/` 目录。

### 2. 修改启动参数

```bash
ros2 launch car_camera camera_apriltag.launch.py \
    camera_name:=logitech_c270 \
    camera_info_url:=file:///home/yangqingxi/rviz_ros2/smartcar/src/car_camera/config/ost.yaml \
    tag_size:=0.119 \
    enable_follower:=true
```

### 3. 确认 TF 外参

相机在车上的安装位置通过 `camera.launch.py` 中的 `cam_x/cam_y/cam_z` 参数指定，需按实际安装位置修改。

### 4. 与底盘联动

```bash
# 终端1：启底盘（含里程计）
ros2 launch car_bringup bringup.launch.py

# 终端2：启摄像头 + 检测 + 视觉伺服
ros2 launch car_camera camera_apriltag.launch.py \
    camera_name:=logitech_c270 \
    camera_info_url:=file:///path/to/ost.yaml \
    enable_follower:=true
```

## 安全特性

- **丢失停车**：标签离开视野超过 0.5 秒 → 自动发零速
- **200ms 看门狗**：car_base 节点收不到 `/cmd_vel` 超过 200ms → 自动停车
- **速度上限**：线速度 ≤ 0.08 m/s，角速度 ≤ 0.5 rad/s
- **死区保护**：到达目标附近（±3mm 横向, ±5mm 深度）→ 停止，防止抖动
- **Ctrl+C 安全**：退出时自动发布零速指令