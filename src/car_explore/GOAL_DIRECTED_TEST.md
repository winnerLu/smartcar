# 目标偏置在线导航测试

这个实验不再要求完整覆盖地图。终点使用小车启动瞬间的车体坐标表示：

- `goal_forward`：车头前方为正，单位 m。
- `goal_left`：车体左侧为正、右侧为负，单位 m。
- `goal_radius`：进入终点附近多大范围即结束，第一轮建议 `0.45` m。即使进入
  这个半径，节点也只会在当前位置到目标之间的栅格均为已知自由空间时结束；
  中间隔墙不会误判到达。

节点先朝目标方向选择可达 Frontier；终点进入已知自由区、且 Nav2 给出的路径
至少 85% 位于已知区域后，才直接导航到终点。失败候选会暂时加入黑名单；没有
可用候选时会沿成功路标回退，避免持续尝试同一个死胡同。

## 香橙派编译

```bash
cd ~/robot_ws
git fetch origin
git switch experiment/goal-directed-navigation
git pull --ff-only

source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j1"
export CMAKE_BUILD_PARALLEL_LEVEL=1

colcon build --symlink-install \
  --packages-up-to car_explore \
  --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF

source ~/robot_ws/install/setup.bash
```

这个分支不包含 Roadmap Explorer，因此不需要重新编译耗时 30 分钟的
`roadmap_explorer`。

## 运行

终端 1 启动底盘、TF 和雷达：

```bash
source ~/robot_ws/install/setup.bash
ros2 launch car_bringup bringup.launch.py use_safety:=false
```

终端 2 一键启动 SLAM、Nav2 和任务节点。下面例子表示终点在车头前方 3.2m、
右侧 0.6m：

```bash
source ~/robot_ws/install/setup.bash
ros2 launch car_explore autonomous_goal_navigation.launch.py \
  goal_forward:=3.2 \
  goal_left:=-0.6 \
  goal_radius:=0.45
```

也可以像之前一样分开启动，便于分别保存日志：

```bash
# 终端 2
ros2 launch car_navigation slam_navigation.launch.py

# 终端 3
ros2 launch car_explore goal_directed_navigation.launch.py \
  goal_forward:=3.2 goal_left:=-0.6 goal_radius:=0.45
```

## 第一轮重点观察

1. 日志记录的绝对目标是否符合初始朝向；启动后不要再搬动车。
2. `/goal_directed_explorer/target_pose` 是最终目标，
   `/goal_directed_explorer/selected_waypoint` 是当前 Frontier/直达点。
3. 进入死胡同时应先由 Nav2 做一次有界恢复；同一候选失败后应换点，全部不可达
   时日志应出现 `previous breadcrumb` 并后撤到上一段已通过区域。
4. 如果过于绕路，先把 `primary_cone_deg` 从 60 增到 75；如果容易执着于目标方向
   的死路，先降到 45，不要同时修改多项权重。
5. 终点误差较大时先增大 `goal_radius`，摄像头到货后再在这个半径内切换泊车。
