# 停车板打印与相机标定

## 停车板

实际使用的图纸是 `parking_board_A3.pdf`，其中包含官方
`tag36h11` ID 0–7：

```text
0  1  2
7     3
6  5  4
```

打印必须选择“实际大小 / 100%”，不能选择适合页面。沿 297×300 mm
裁切线剪下后，用尺测量任意标签的黑色检测方框外缘，应为
50 mm；检测器参数对应 `tag_size:=0.050`。四角和四边中点标签的
黑框贴板边，因此停车板外部应是浅色、无复杂纹理的地面，以提供静区。

如果重新生成图纸：

```bash
python3 gen_8tags.py
python3 gen_a3_pdf.py
```

生成脚本内置官方前八个 codeword，并有测试防止标签 ID 或打印尺度
再次发生偏差。

## 相机标定（新手完整流程）

标定的目的是获取相机的**内参**（fx, fy, cx, cy）和**畸变系数**。没有内参，AprilTag 算出来的距离和角度会偏很多。

当前 `webcam_calibration.yaml` 里的 fx=fy=457 是按 70° 视场角估算的近似值，只能验证流程，不能用于精确控制。

## 1. 打印棋盘格

棋盘格是 9×6 个**内角点**（黑块十字交叉处），不是格子数。

```bash
wget https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png \
    -O ~/checkerboard.png
```

打印要求：

- 纸张：A4
- 打印对话框必须选**「实际大小 / 100%」**，禁止勾选「适合页面」或任何缩放选项
- 打印后用**尺实测每个方格边长**（比如理论 25mm 实际量出来是 24.5mm，就填 0.0245）
- 建议贴在硬纸板或亚克力板上，保持**绝对平整**，弯折会导致标定失败

## 2. 安装标定工具

```bash
sudo apt install -y ros-humble-camera-calibration
```

## 3. 启动摄像头

新开终端，**只要摄像头，不要检测节点**：

```bash
cd ~/rviz_ros2/smartcar
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch car_camera camera.launch.py video_device:=/dev/video0
```

## 4. 运行标定器

另开终端：

```bash
ros2 run camera_calibration cameracalibrator \
    --size 9x6 \
    --square 0.025 \
    --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

| 参数 | 说明 | 示例 |
|------|------|------|
| `--size 9x6` | 内角点数（宽×高） | 标准 OpenCV 棋盘就是 9×6 |
| `--square 0.025` | 方格实测边长（米） | 量出来 24.5mm 就填 0.0245 |

## 5. 采集样本

标定器窗口打开后，把棋盘对着摄像头**缓慢移动**，覆盖所有方向和角度：

| 方向 | 怎么动 |
|------|--------|
| **X** | 棋盘移到画面**最左边** → **最右边** |
| **Y** | 棋盘移到画面**最上方** → **最下方** |
| **Z** | 棋盘**靠近**摄像头 → **远离**（覆盖 10cm~100cm 范围） |
| **Size** | 棋盘离近占满画面 → 离远只占一小块 |
| **Skew** | 棋盘**左右倾斜** 30° → **上下倾斜** 30° |

窗口中会显示四条进度条（X / Y / Size / Skew），**全部变绿**之后 CALIBRATE 按钮会亮起。

采集注意事项：

- 棋盘必须**绝对平整**，不能弯折、不能卷边
- 避免光照反光，室内灯光从侧面打，不要直射棋盘
- 每个位置停留 1~2 秒，让它稳定采集 2~3 帧
- 棋盘**始终完整在画面内**，四角不能出画
- 约采集 **30~50 张**不同姿态就足够了
- 如果标定结果误差大（epipolar error > 0.5），删掉模糊/反光的样本重标

## 6. 计算并保存

- 点 **CALIBRATE** → 等待几秒运算（终端会显示标定误差）
- 点 **SAVE** → 标定数据打包到 `/tmp/calibrationdata.tar.gz`
- 点 **COMMIT** → 通知 camera_info_manager 更新参数

解压并放入项目：

```bash
cd ~/rviz_ros2/smartcar/src/car_camera/config
tar xzf /tmp/calibrationdata.tar.gz ost.yaml
mv ost.yaml c270_calibration.yaml
```

## 7. 验证标定效果

重启并检查内参：

```bash
ros2 launch car_camera camera_apriltag.launch.py \
    video_device:=/dev/video0 \
    camera_info_url:=file:///home/yangqingxi/rviz_ros2/smartcar/src/car_camera/config/c270_calibration.yaml \
    camera_name:=logitech_c270
```

查看日志：

```
[apriltag_detector]: 相机内参已获取: fx=623.4 fy=621.8 cx=318.2 cy=242.7
```

如果 fx/fy 不再是 457（近似估算值），而是 500~700 的实测值，说明标定成功。
