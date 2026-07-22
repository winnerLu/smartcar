# WFD + MRTSP 探索测试

本实验仅替换探索目标生成与排序节点。SLAM、Nav2、A* 全局规划、DWB
局部控制、车体 footprint 和速度限制均沿用当前配置。第一轮不使用相机，也不加入
初始方向偏置，目标是验证全局自主探索、重复路径和完成返航能力。

## 拉取并构建依赖

```bash
cd ~/robot_ws
vcs import src < explore.repos
rosdep install --from-paths src --ignore-src -r -y

# 香橙派内存有限，依赖包首次构建使用单线程并关闭测试。
export MAKEFLAGS="-j1"
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --symlink-install --packages-up-to car_explore \
  --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`frontier_exploration_ros2` 固定为 v1.6.1 对应提交，不要同时运行原来的
`explore_lite`，否则两个探索节点会持续抢占 Nav2 目标。

## 实机启动

终端1启动底盘、雷达与紧急碰撞兜底（实验分支默认
`use_safety:=true`）：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source ~/sdk_ldrobotsensorteam_stl/ros2_app/install/setup.bash
source install/setup.bash
ros2 launch car_bringup bringup.launch.py
```

终端2启动 SLAM + Nav2：

```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch car_navigation slam_navigation.launch.py
```

看到 `Managed nodes are active` 后，在终端3运行：

```bash
ros2 launch car_explore mrtsp_explore.launch.py
```

启动后先不下发目标，手动低速前进确认 `/cmd_vel_safe` 能输出、
`/collision_monitor_state` 有数据，并确认 RViz/Foxglove 中雷达点与
`/local_costmap/published_footprint` 对齐。这一版碰撞监控只做 0.4s 紧急预测，
Nav2 局部代价地图和 DWB 仍是主要避障手段。

恢复顺序已改为“清图→后退10cm→左60°→从左侧跨过原航向到右60°”。
如果终端出现 `Collision Ahead` 而实际有空间，先保存当时的局部
costmap 和 footprint，不要直接关闭安全链继续无人测试。

探索节点自然耗尽有效 Frontier 后会向启动时记录的位姿发送返航目标。
`/exploration_complete` 表示探索流程完成；返航结果仍应结合终端日志确认。

## 第一轮记录项

- 建图完成总时间和返航是否成功；
- 里程计总路程及明显重复经过区域；
- `NavigateToPose` 目标数量、失败次数和长时间停滞；
- 香橙派 CPU、内存，以及 Nav2 是否出现 missed rate；
- `/explore/frontiers`、`/explore/selected_frontier` 和
  `/explore/optimized_map` 是否符合实际空闲区域。

对照测试应使用同一场地、同一起点和相近障碍摆放，分别运行当前
`explore.launch.py` 与本文件至少三次。
