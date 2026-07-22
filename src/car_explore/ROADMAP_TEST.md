# Roadmap Explorer 实机实验

本实验分支基于可运行版本 `main@dabe0dc`，引入
[`suchetanrs/roadmap-explorer`](https://github.com/suchetanrs/roadmap-explorer)
的固定提交 `203ff2602e1a1e41eec7ef663eacc21266e270c6`。

## 与上游默认配置的区别

- 使用 ROS 2 Humble 和 CycloneDDS；上游已知 FastDDS 存在生命周期重启崩溃问题。
- 运行在 `slam_toolbox` 在线建图模式，探索代价地图直接订阅 `/map`。
- 使用实测不对称 footprint，不允许上游把全局代价地图覆盖成 0.10m 圆形机器人。
- 搜索半径、roadmap 网格和信息增益范围缩小到几米级赛场。
- 全局重规划频率由上游 10Hz 降为 1Hz，避免与 SLAM 一起压满香橙派。
- `roadmap_explore_mission.py` 在开始时记录 `map -> base_link`，探索正常完成后调用
  Nav2 返回该位姿；探索异常退出时不会盲目返航。

## 香橙派依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-pcl-ros \
  ros-humble-nav2-navfn-planner \
  ros-humble-nav2-map-server \
  ros-humble-nav2-behavior-tree \
  ros-humble-nav2-theta-star-planner \
  ros-humble-behaviortree-cpp-v3 \
  nlohmann-json3-dev \
  libyaml-cpp-dev \
  libboost-dev
```

为避免香橙派编译内存不足，关闭测试并单线程构建：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j1"
export CMAKE_BUILD_PARALLEL_LEVEL=1

colcon build --symlink-install \
  --packages-up-to roadmap_explorer car_explore \
  --executor sequential \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DBUILD_COMPARISON_TOOLS=OFF
```

## 启动

三个终端都要设置 CycloneDDS：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

终端 1（先测试不启用外层碰撞监控，与昨天基线一致）：

```bash
ros2 launch car_bringup bringup.launch.py use_safety:=false
```

终端 2：

```bash
ros2 launch car_navigation slam_navigation.launch.py
```

终端 3：

```bash
ros2 launch car_explore roadmap_exploration.launch.py
```

若只想观察算法，不自动返航：

```bash
ros2 launch car_explore roadmap_exploration.launch.py return_to_start:=false
```

## 首次测试关注项

- `roadmap_explorer_costmap` 是否订阅 `/map` 并进入 current 状态。
- 是否持续产生 roadmap/frontier 标记和 Nav2 `navigate_to_pose` 目标。
- `planner_server` 负载是否仍能保持 1Hz 规划，不出现连续超时。
- 目标切换是否比 `explore_lite` 少，重复路径是否减少。
- 无前沿时 action 是否以 `NO_MORE_REACHABLE_FRONTIERS` 成功结束并返航。

这版不会修改 `main`；切回基线只需 `git switch main` 并重新构建
`car_explore`、`car_navigation`。
