# smartcar ros2_ws

司南驭途·智泊九州 迷宫泊车挑战赛 — ROS2 工作空间。

## 平台
- ROS2 Humble
- 运行环境:Orange Pi 4A (arm64),Ubuntu 22.04
- 开发环境:WSL2 (x86_64),仅编码与编译验证

## 构建
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> `build/` `install/` `log/` 为编译产物,含架构相关二进制与绝对路径,不纳入版本控制。
> **跨机器同步后必须在目标机重新 `colcon build`,不要拷贝 build/install。**

## 一键启动目标导向探索

脚本会依次启动底盘与雷达、SLAM/Nav2、Roadmap 目标探索和视觉泊车，
并在 `test_runs/testN/logs/` 保存三个终端的日志。

交互输入目标：

```bash
cd ~/robot_ws
./start_roadmap_mission.sh
```

也可以直接传入“前方、左侧”坐标（单位：米，右侧为负）：

```bash
./start_roadmap_mission.sh 2.0 8.2
```

若检测到上一轮测试仍在运行，脚本默认拒绝启动第二套控制节点。确认小车安全
且需要替换旧会话时使用：

```bash
./start_roadmap_mission.sh --replace 2.0 8.2
```

启动后会自动进入目标任务的 tmux 窗口。按 `Ctrl+b`、再按 `d` 可退出界面但
保持任务运行。

## 功能包
- `hello_pkg` — 初始验证包(占位)
- 后续:car_base / car_bringup / car_description / car_slam / car_nav / car_explore / car_perception / car_parking / car_mission
