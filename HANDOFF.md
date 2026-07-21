# smartcar 项目交接文档

> 面向接手开发的 agent / 队友。先读本文件 + `COMMANDS.md`(命令速查)+ `docs/总体方案设计_v1.3.md`(完整方案,在仓库外的 docs/,若能访问)。
> 最后更新:2026-07-21

---

## 0. 一句话项目背景

智能车比赛「司南驭途·智泊九州」迷宫泊车挑战赛。差速小车在未知场地边建图边导航,到达终点区(+60分),自主停入 30×30cm 泊车区(+40分),限时 5 分钟,撞一次罚 30 秒。
比赛策略:起点人工把车头朝向终点大致方向(定为 map +X),方向引导探索 → 发现自放的高位颜色球+四面 AprilTag 信标 → 视觉泊车。

---

## 1. 两台机器的分工(重要)

| 机器 | 角色 | 说明 |
|---|---|---|
| **WSL(x86_64)** | 开发机 | 写代码、编译验证、git 推送。**无硬件**(无串口/雷达/摄像头),不能实机测试 |
| **香橙派 Orange Pi 4A(arm64)** | 实机运行 | ROS2 Humble,接底盘/雷达/摄像头。真正跑和测都在这 |

**跨架构铁律**:
- 只提交源码,**绝不提交 build/install/log**(已在 .gitignore)。二进制架构相关,拷过去用不了。
- 同步后**在目标机重新 `colcon build`**,不能拷 build/install。
- 代码走 GitHub 中转:WSL push → 香橙派 pull → colcon build。

**远程仓库**:`git@github.com:winnerLu/smartcar.git`(公司 GitLab 被安全网关挡,改用 GitHub)。
仓库根 = `ros2_ws/`(即工作空间根就是 git 根)。

---

## 2. WSL 端注意事项

- 工作目录:`/home/lsc/devs/smartcar/`;工作空间:`smartcar/ros2_ws/`(git 根)。
- 文档在 `smartcar/docs/`,**在 git 仓库外**,不随代码同步(方案文档、标定记录都在这)。
- WSL 能编译所有包(纯逻辑验证),但**跑不了需要硬件的节点**(串口/雷达/摄像头)。
- WSL 与香橙派 **ROS2 DDS 不通**(WSL2 NAT + 公司网关),不能跨机看话题。可视化靠香橙派跑 foxglove_bridge + 浏览器。
- 编译命令:`source /opt/ros/humble/setup.bash && colcon build --symlink-install`
- 改代码前**先 `git pull`**——队友(摄像头 car_camera)会推代码,防冲突。
- 提交规范:使用简洁中文提交信息，不添加 `Co-Authored-By`。文档改动在 docs/(仓库外),不进 git。

---

## 3. 香橙派端注意事项

- 工作空间:`~/robot_ws`(git clone 的);雷达 SDK 独立工作空间:`~/sdk_ldrobotsensorteam_stl/ros2_app`。
- `~/.bashrc` 已自动 source:humble + robot_ws + 雷达SDK。新终端直接可用 ros2。
- **两个工作空间都要 source**(bashrc 已配)。雷达包 `ldlidar` 在 SDK 那个空间。

**硬件坑(务必知道)**:
- **雷达供电敏感**:STL-19P 电机启动电流大,插到供电不足的 USB 口会「connection normal 后一出数据就 abnormal」秒退。**必须插供电足的 USB 口**。多设备同开时更易掉。反复出现就换口。
- **串口设备名用 udev 固定名**:底盘 `/dev/car_base`、雷达 `/dev/ldlidar`(规则 `car_bringup/config/99-smartcar.rules`,已部署到 `/etc/udev/rules.d/`)。别用 ttyACM/ttyUSB(会漂移)。
- **App 与 car_base 冲突**:小车有蓝牙 App 能控车,但 car_base 一跑就抢串口指令,App 控不了。用 ROS 的 /cmd_vel 控车,别混用 App。
- **别同时开建图和导航**:slam.launch.py 和 navigation.launch.py 都发 map→odom,冲突。联动用 slam_navigation.launch.py。

**依赖**:若 colcon build 报缺包,`rosdep install --from-paths src --ignore-src -y`。已知需要:slam_toolbox、navigation2、nav2_bringup、twist_mux、nav2_collision_monitor(基本都装了)。

---

## 4. 当前进度(已完成 / 验证过)

**硬件层 ✅**(car_base):
- STM32 串口通信(11字节控制帧 / 24字节反馈帧,大端 BCC 校验)。**控制帧布局:[帧头][2预留位][X速度][Y速度][Z角速度][校验][帧尾],无 flag_stop**(曾误加 flag_stop 致字节错位、方向随速度翻转,已修)。
- 里程计(积分车体 vx/wz)+ TF odom→base_link + IMU(/imu/data_raw) + 电池(/battery)。
- 标定完成:cmd_vx_sign/cmd_wz_sign/odom_vx_sign/odom_wz_sign **全 +1**;odom_wz_scale=**1.125**(线速度准);IMU 直接透传(重力Z、gyro逆时针为正,与odom同号)。
- footprint 实测**不对称**(轮子在车尾):`[[0.197,0.093],[0.197,-0.093],[-0.033,-0.093],[-0.033,0.093]]`,padding 0.02。
- 键盘遥控 `keyboard_teleop.py`(死人开关,松手0.4s停,限幅)。

**雷达 ✅**:ldlidar 出 /scan(10Hz),frame=base_laser。外参 static TF 由 bringup 发:x=0.16 y=-0.01 z=0.115 **yaw=4.10**(含雷达上壳装错180°补偿;若上壳转正改回~0.96)。laser_scan_dir=True 无镜像。

**建图 ✅**(car_slam):slam_toolbox async 在线建图,实测正常。

**导航 ✅**(car_navigation):
- 静态图导航(navigation.launch.py + AMCL):实测通过。
- **SLAM+Nav2 联动(slam_navigation.launch.py):比赛模式,实测点目标能导航避障**。
- 算法:NavFn(Dijkstra)全局 + DWB 局部 + 分层 costmap + AMCL(静态图时)。

**安全链 ✅**(car_navigation):twist_mux + collision_monitor。`bringup use_safety:=true` 一键启用。防撞实测生效(近障降速/停车)。

**摄像头**(car_camera,队友写):罗技 C270 采集节点已在仓库,**尚未联调对接**(AprilTag/颜色球视觉链路)。

---

## 5. 待解决 / 下一步(按优先级)

1. **自主探索 explore_lite**(m-explore-ros2)——已跑通「Frontier选择→Nav2移动→地图扩展」闭环；`min_frontier_size` 已由 0.75m 调为 0.30m，并启用自然完成后返回启动位姿，**待实机复测完成判定与返航**。上游源码固定在 `explore.repos`;优化版(P3)再自研方向偏置(朝 map +X)。
2. **防撞脱困**(和探索一起调):进度检查已由 0.5m/15s 改为 0.10m/10s。恢复树现为清图→左转→右转→受限倒车(0.10m@0.05m/s)→短等；正常 DWB 仍禁倒车。实测固定前后 stop/slowdown 区会把远离障碍的恢复动作也归零，已改为基于 `/local_costmap/published_footprint` 的方向感知 approach 模型；因安全链介入仍偏多，当前采用激进兜底参数（TTC 0.6s，至少6个激光点触发），**待实机复测并对比 `/cmd_vel_raw` 与 `/cmd_vel_safe`**。
3. **EKF**(robot_localization 融合轮式+IMU):抗碰撞/打滑里程计漂移。方案要求上,目前用纯轮式够用。上了要把 car_base 的 publish_tf 关掉、Nav2 odom_topic 改回 /odometry/filtered。
4. **视觉泊车**(car_parking):/parking_hint 解耦接口(可人工发调试)→ 导航到附近 → 四面 AprilTag → 视觉闭环停车。依赖摄像头联调。

**Nav2 参数微调**(到点行为/窄道/速度)留待整体联调时统一调,现在够用。

---

## 6. 关键约定 / 踩过的坑(避免重蹈)

- **AMCL 初始位姿必须用 `--qos-reliability reliable` 发**,否则 AMCL 收不到(普通 pub 显示发了但无效)。
- **nav2_params.yaml 必须有 map_server 段**(否则 yaml_filename 空、localization 起不来)。
- **collision_monitor 的 polygon points 是扁平 double 数组**(`[x1,y1,x2,y2,...]`),不能写字符串。
- **建图要慢 + 走回环**,否则地图旋转错位(第一次建图踩过)。
- **SLAM 抗不了手动搬车**(绑架问题),比赛车不会被搬,正常导航够用。
- **ros2 run 找 .py 脚本**:脚本已加执行权限;若仍 No executable found,直接 `python3 install/car_base/lib/car_base/xxx.py`。

---

## 7. 快速上手命令(详见 COMMANDS.md)

```bash
# 香橙派:更新+编译
cd ~/robot_ws && git pull && colcon build --symlink-install && source install/setup.bash

# 遥控(车架空测试)
ros2 launch car_bringup bringup.launch.py use_lidar:=false
ros2 run car_base keyboard_teleop.py    # w/s前后 a/d转 空格停

# 建图
ros2 launch car_bringup bringup.launch.py      # 终端1
ros2 launch car_slam slam.launch.py            # 终端2
ros2 run car_base keyboard_teleop.py           # 终端3 遥控建图
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/图名   # 存图

# SLAM+Nav2 联动(比赛模式,推荐)
ros2 launch car_bringup bringup.launch.py               # 终端1
ros2 launch car_navigation slam_navigation.launch.py    # 终端2
ros2 run foxglove_bridge foxglove_bridge                # 终端3 可视化
# Foxglove(浏览器 studio.foxglove.dev 允许不安全内容连 ws://香橙派IP:8765)点 2D Nav Goal

# 带防撞的完整启动
ros2 launch car_bringup bringup.launch.py use_safety:=true

# 静态图导航(需先建图存下)
ros2 launch car_navigation navigation.launch.py map:=/绝对路径/图.yaml
# 发初始位姿(必须 reliable):见 COMMANDS.md
```

---

## 8. 包结构

```
ros2_ws/src/
├── car_base/        C++,底盘串口通信+里程计+TF+IMU;含遥控/调试脚本(scripts/)
├── car_bringup/     启动编排(bringup.launch.py 一键起硬件,use_safety 开关)+ udev 规则
├── car_slam/        slam_toolbox 建图配置
├── car_navigation/  Nav2配置+多个launch(静态图/联动/安全链)+ twist_mux/collision_monitor 配置
├── car_explore/     explore_lite参数+自主探索组合launch(待实机联调)
├── car_camera/      队友:罗技C270摄像头采集(待联调)
└── hello_pkg/       初始占位包(可忽略)
```

标定值权威记录在 `docs/protocols/car_base_标定记录.md`(仓库外)。
