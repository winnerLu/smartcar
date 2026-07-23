# 目标导向 Roadmap Explorer 实机实验

本实验分支基于可运行版本 `main@dabe0dc`，引入
[`suchetanrs/roadmap-explorer`](https://github.com/suchetanrs/roadmap-explorer)
的固定提交 `203ff2602e1a1e41eec7ef663eacc21266e270c6`。

## 与上游默认配置的区别

- 使用 ROS 2 Humble 和 CycloneDDS；上游已知 FastDDS 存在生命周期重启崩溃问题。
- 运行在 `slam_toolbox` 在线建图模式，探索代价地图直接订阅 `/map`。
- 使用实测不对称 footprint，不允许上游把全局代价地图覆盖成 0.10m 圆形机器人。
- 搜索半径、roadmap 网格和信息增益范围缩小到几米级赛场。
- 全局重规划频率由上游 10Hz 降为 1Hz，避免与 SLAM 一起压满香橙派。
- 根据启动时车头方向和终点相对坐标，优先选择向终点取得进展的 Frontier，
  而不是要求扫完整张地图。
- 目标方向暂时不可达时依次放宽到±60°、±120°和全方向，因此可以绕出死胡同。
- 只有目标成为已知自由区、半径内安全落点与目标之间无墙，且 Nav2 能规划出
  至少 85% 位于已知自由区的路径时，才会停止 Roadmap 探索并转入最终导航。
- 恢复顺序为清图、低速后退 0.10m、左转60°、转到右60°，避免旧策略左右
  旋转后又回到原姿态。

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

后续只更新本分支时不要加 `--cmake-clean-cache`，增量编译会比首次快很多。

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

终端 3（例如终点在初始车头前方 3.2m、右侧 0.6m）：

```bash
ros2 launch car_explore roadmap_exploration.launch.py \
  goal_forward:=3.2 \
  goal_left:=-0.6 \
  goal_radius:=0.25
```

坐标定义：

- `goal_forward`：启动瞬间沿车头向前为正，单位 m。
- `goal_left`：车体左侧为正、右侧为负，单位 m。
- `goal_radius`：允许在标称终点附近停车的半径。建议无摄像头时用 `0.20~0.25`；
  它不是单纯欧氏距离判断，中间有墙或未知格子时不会触发完成。

如果需要对照上游 Roadmap 的完整探索行为：

```bash
ros2 launch car_explore roadmap_exploration.launch.py \
  goal_directed_mode:=false \
  return_to_start:=true
```

## 首次测试关注项

- `roadmap_explorer_costmap` 是否订阅 `/map` 并进入 current 状态。
- 是否持续产生 roadmap/frontier 标记和 Nav2 `navigate_to_pose` 目标。
- `planner_server` 负载是否仍能保持 1Hz 规划，不出现连续超时。
- 目标切换是否比 `explore_lite` 少，重复路径是否减少。
- 是否先出现 `Goal-directed Roadmap selected frontier`，目标方向被堵后才选侧向 Frontier。
- 只有出现 `Target has a safe path ... switching to final Nav2 navigation` 后才结束探索。
- 指定点与小车很近但中间隔墙时，是否继续探索，这是本版最重要的安全验证。

这版不会修改 `main`；切回基线只需 `git switch main` 并重新构建
`car_explore`、`car_navigation`。
