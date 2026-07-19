# 导航静态地图

把 slam_toolbox 建好并存下的地图(`.pgm` + `.yaml`)放这里,例如 `test1.pgm` / `test1.yaml`。

存图命令(slam 运行时):
```bash
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/car_navigation/maps/赛场图
```

启动导航时指定:
```bash
ros2 launch car_navigation navigation.launch.py map:=<地图yaml绝对路径>
```

注意:navigation.launch.py 默认找 `test1.yaml`,没有的话必须用 map:= 指定实际地图。
