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

## 手动 source(若新终端找不到包)

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
source ~/sdk_ldrobotsensorteam_stl/ros2_app/install/setup.bash
```

## 关闭节点

`Ctrl+C`。遥控节点退出会自动发停车指令。
