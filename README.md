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

## 功能包
- `hello_pkg` — 初始验证包(占位)
- 后续:car_base / car_bringup / car_description / car_slam / car_nav / car_explore / car_perception / car_parking / car_mission
