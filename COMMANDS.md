# smartcar 常用命令速查(香橙派)

> 环境已在 ~/.bashrc 自动 source:humble + robot_ws + 雷达SDK。
> 新终端直接可用 ros2;若报找不到包,先手动 source(见文末)。

## 1. 更新代码 + 编译

```bash
cd ~/robot_ws
git pull                                          # 拉取 WSL 上推送的最新代码
colcon build --symlink-install                    # 编译全部包
# 只编改动的包更快:
colcon build --packages-select car_base car_bringup
source install/setup.bash                         # 编译后刷新环境(当前终端)
```

> 雷达 SDK 是独立工作空间,一般不用重编;若改了它:
> `cd ~/sdk_ldrobotsensorteam_stl/ros2_app && colcon build --packages-select ldlidar`

## 2. 一键启动(底盘 + 雷达 + TF)

```bash
ros2 launch car_bringup bringup.launch.py         # 全部启动
ros2 launch car_bringup bringup.launch.py use_lidar:=false   # 只启底盘,不启雷达
```

启动后有:/scan(雷达10Hz)、/wheel/odom(底盘20Hz)、/imu/data_raw(20Hz)、
TF: odom->base_link->base_laser。

## 3. 遥控小车(车务必架空或留足空间)

```bash
# 自写遥控(推荐:固定档位+死人开关,松手0.4s自动停)
ros2 run car_base keyboard_teleop.py
#   w/s 前进后退  a/d 左右转  空格/x 停  =/- 调线速度  ]/[ 调角速度  q 退出

# 官方遥控(功能全,q/z无上限靠car_base限幅兜底)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
#   i前进 ,后退 j/l左右转 k停  q/z调速
```

## 4. 查看数据 / 调试

```bash
ros2 topic list                          # 所有话题
ros2 topic hz /scan                      # 雷达频率(应~10Hz)
ros2 topic hz /wheel/odom                # 里程计频率(应~20Hz)
ros2 topic echo /wheel/odom --field twist.twist   # 看车速反馈
ros2 topic echo /imu/data_raw            # 看IMU
ros2 topic echo /battery                 # 电池电压(未标定)
ros2 node list                           # 运行中的节点
ros2 run tf2_ros tf2_echo base_link base_laser    # 查雷达外参TF
ros2 run tf2_tools view_frames           # 生成TF树pdf
```

## 5. 串口通信底层调试(不经ROS,需先停掉car_base)

```bash
cd ~/robot_ws
# 只监听STM32反馈帧
python3 src/car_base/scripts/serial_check.py --port /dev/car_base --listen
# 发指令测试(架空!)
python3 src/car_base/scripts/serial_check.py --port /dev/car_base --vx 0.1 --duration 2
# 看发送帧十六进制
python3 src/car_base/scripts/serial_check.py --port /dev/car_base --vx 0.1 --hex --duration 2
```

car_base 节点也可开发送帧调试:
```bash
ros2 run car_base car_base_node --ros-args -p port:=/dev/car_base -p debug_tx:=true
```

## 6. 设备 / 权限排查

```bash
ls -l /dev/car_base /dev/ldlidar          # udev固定软链接(应指向ttyACM*/ttyUSB*)
ls -l /dev/ttyACM* /dev/ttyUSB*           # 实际串口设备
```

## 7. 参数覆盖示例(不改配置文件临时试)

```bash
# 覆盖速度符号(布局修正后若发现方向反)
ros2 launch car_bringup bringup.launch.py   # 参数在 config/car_base.yaml
# 单独起节点覆盖参数:
ros2 run car_base car_base_node --ros-args \
  -p port:=/dev/car_base -p cmd_vx_sign:=1 -p cmd_wz_sign:=1
# 遥控降速:
ros2 run car_base keyboard_teleop.py --ros-args -p linear_speed:=0.08
```

## SLAM 建图(边走边建)

三/四个终端:
```bash
# 终端1: 硬件
ros2 launch car_bringup bringup.launch.py
# 终端2: 建图(slam_toolbox)
ros2 launch car_slam slam.launch.py
# 终端3: 遥控(慢速平稳,走回环,别急转)
ros2 run car_base keyboard_teleop.py     # 若 ros2 run 找不到,用: python3 install/car_base/lib/car_base/keyboard_teleop.py
# 终端4: 可视化
ros2 run foxglove_bridge foxglove_bridge
```
Foxglove: Fixed frame=map,开 /map /scan /tf。建图要慢、走回环,否则地图会旋转错位。

存图(slam 运行时任意时刻):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/图名
```

> 注意:建图(slam.launch.py)和导航(navigation.launch.py)不能同时开,都发 map->odom 会冲突。

## Nav2 静态图导航

```bash
# 终端1: 硬件
ros2 launch car_bringup bringup.launch.py
# 终端2: 导航(必须 map:= 指定实际地图绝对路径)
ros2 launch car_navigation navigation.launch.py map:=/home/orangepi/robot_ws/maps/图名.yaml
# 终端3: 可视化
ros2 run foxglove_bridge foxglove_bridge
```

**发初始位姿(关键!必须带 --qos-reliability reliable,否则 AMCL 收不到)。**
车放建图起点、朝向 map +X(0°):
```bash
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}' --qos-reliability reliable
```
朝向 45° 则 orientation 用 `{z: 0.3827, w: 0.9239}`;90° 用 `{z: 0.707, w: 0.707}`。

验证定位:
```bash
ros2 run tf2_ros tf2_echo map odom     # 有输出=map->odom通了
```
Foxglove 看 /scan 点云和 /map 墙重合=定位准 → 用 RViz/Foxglove "2D Nav Goal" 点目标导航。

**导航注意**:别开 keyboard_teleop(抢 /cmd_vel);留足空间准备急停;窄道规划不出路径多为 inflation_radius(0.25) 偏大,实测再调。

## 自主探索(explore_lite)

首次部署或 `explore.repos` 版本变化后，在香橙派安装源码依赖：

```bash
cd ~/robot_ws
vcs import src < explore.repos
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

> 若没有 `vcs` 命令：`sudo apt install python3-vcstool`。`m-explore-ros2` 已在
> `explore.repos` 固定提交；不要复制 WSL 的 build/install 到香橙派。

实机启动（先架空确认节点正常，再放到空旷场地）：

```bash
# 终端1：底盘 + 雷达 + 速度安全链
# 可选外层碰撞监控；当前自主探索推荐先使用默认 use_safety:=false，依靠 Nav2 避障。
ros2 launch car_bringup bringup.launch.py use_safety:=true

# 终端2：在线SLAM + Nav2 + explore_lite（启动后会自动开始探索）
ros2 launch car_explore autonomous_exploration.launch.py

# 终端3：可视化
ros2 run foxglove_bridge foxglove_bridge
```

暂停、恢复和观察：

```bash
ros2 topic pub -1 /explore/resume std_msgs/msg/Bool '{data: false}'  # 暂停并取消目标
ros2 topic pub -1 /explore/resume std_msgs/msg/Bool '{data: true}'   # 恢复探索
ros2 topic echo /explore/status
ros2 action info /navigate_to_pose
```

Foxglove 使用 `map` 固定坐标系，显示 `/map`、`/scan`、TF 和
`/explore/frontiers`。探索期间不要运行键盘遥控，也不要再启动独立的
`slam.launch.py` 或 `slam_navigation.launch.py`。

## 常见坑

- AMCL 初始位姿:必须 `--qos-reliability reliable`,普通 pub 显示发了但 AMCL 收不到。
- map_server 报 `yaml_filename is not initialized`:nav2_params.yaml 需有 map_server 段(已修)。
- 雷达 `communication is abnormal` 秒退:供电不足,换香橙派供电足的 USB 口。
- 设备名漂移:用 udev 固定名 /dev/car_base、/dev/ldlidar(已配)。

## 手动 source(若新终端找不到包)

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
source ~/sdk_ldrobotsensorteam_stl/ros2_app/install/setup.bash
```

## 关闭节点

`Ctrl+C`。遥控节点退出会自动发停车指令。
