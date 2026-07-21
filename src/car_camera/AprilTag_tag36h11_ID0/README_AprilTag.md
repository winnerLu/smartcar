# AprilTag `tag36h11` #0 — 使用说明

本目录提供一张 **AprilTag `tag36h11` 家族、ID 0** 的标签，可直接用于视觉定位、位姿估计、机器人泊车等场景。图像按官方 `AprilRobotics/apriltag` 的 codebook 与渲染规则本地生成，**内容与官方 `apriltag-imgs` 仓库中的 `tag36_11_00000.png` 完全一致**（已做位级回读校验）。

---

## 1. 规格一览

| 项目 | 值 |
|---|---|
| 家族 (family) | `tag36h11` |
| ID | `0` |
| 码字 (codebook) | `0x0d7e00984b` |
| 黑色编码区边长 | **120 mm**（= 填进检测器的 `tag_size`） |
| 含白边总尺寸 | 150 mm（外围 1 格 quiet zone） |
| 网格 | 10×10 格 = 1 白边 + 8×8 标签（1 黑边 + 6×6 数据） |
| 建议用途 | 室内视觉定位 / 视觉泊车 / 单目位姿估计 |

---

## 2. 文件清单与校验

| 文件 | 说明 | SHA256 |
|---|---|---|
| `tag36h11_0_A4_120mm.pdf` | **A4 打印版**（含标尺，黑色方块 120 mm） | `48c5d48263d96321dc540268d80c20dbed837405f0a249d2cb5901bd60accd74` |
| `tag36_11_00000.png` | 纯图，1200×1200 灰度 | `4d38347d311c75db7ad0cf8c47cf0cbdb67abe9eacd8b30efdede06447ce8f3a` |
| `gen_apriltag.py` | PNG 生成脚本（纯标准库，可改尺寸/ID） | `da34551777b34b021f69b14a5c7957fd8c179dd4bcef11ad2e7be88ae9328031` |
| `gen_print.py` | A4 打印 PDF 生成脚本 | `f22b8c55f14c475ccf5905622f59b266e008bbd51935447c2fc215e92690b35c` |

收到文件后可用 `sha256sum <文件>` 核对，确认未被篡改。

---

## 3. 开源来源与许可（务必保留）

本标签的数据与算法来自以下开源项目，**均采用 BSD-2-Clause 许可**，可自由使用、修改、分发（需保留版权声明）。

| 用途 | 仓库 | 链接 |
|---|---|---|
| **codebook / 渲染规则**（本标签的权威来源） | AprilRobotics/apriltag | https://github.com/AprilRobotics/apriltag |
| 官方预渲染 PNG 仓库（Git LFS） | AprilRobotics/apriltag-imgs | https://github.com/AprilRobotics/apriltag-imgs |
| **ROS 2 检测节点**（推荐，Humble） | christianrauch/apriltag_ros | https://github.com/christianrauch/apriltag_ros |
| Python 检测库（任选其一） | pupil-apriltags | https://github.com/pupil-labs/apriltags |
| Python 检测库 | dt-apriltags | https://github.com/duckietown/dt-apriltags |
| Python 检测库 | pyapriltags | https://github.com/PyAprilTags/pyapriltags |

> 本目录中的 `tag36_11_00000.png` 取自 `apriltag` 仓库 `tag36h11.c` 的 `codedata[0] = 0x0d7e00984b`，并按 `apriltag.c` 官方图像生成函数的位序（MSB 对应 i=0）渲染。这也是它内容等同于官方 `apriltag-imgs` 的原因。

---

## 4. 打印与制作（非常关键）

打印环节出错是 AprilTag 失效最常见的原因，请严格按以下步骤：

1. **打印 `tag36h11_0_A4_120mm.pdf`**，打印对话框选择：
   - ✅ **「实际大小 / 100%」**
   - ❌ **不要**勾「适合页面 / Fit to Page / 缩放以适合」
   - 纸张：A4 纵向。
2. **核对尺寸**：量标签下方标尺，0→12 cm 必须正好 12 cm。对不上就重打（关掉缩放）。
3. **量黑色方块实际边长**（例如 119 mm），**这个实测值**就是要填进检测器的 `tag_size`。
4. **材质**：
   - **哑光**纸/贴纸（光面会反光，导致某些角度识别失败）；
   - 贴在 **3~5 mm 刚性背板**（雪弗板 / 亚克力 / 铝板）上，**必须绝对平整**，翘曲直接毁掉位姿精度；
   - 长期使用建议哑光过塑防污。

---

## 5. 在代码里使用

### 5.1 ROS 2（`apriltag_ros`，推荐）

最小配置示例（具体字段名以仓库 README 为准）：

```yaml
apriltag:
  ros__parameters:
    family: 'tag36h11'
    size: 0.119              # ← 用你打印后实测的黑色方块边长(米), 别直接填 0.120
    max_hamming: 2
    image_topic: /camera_front/image_raw
    camera_info_topic: /camera_front/camera_info
    z_up: true
    standalone_tags:
      - name: 'parking_tag'
        id: 0
        size: 0.119
```

- `size` 与 `standalone_tags.size` 要一致，且都用**实测米数**。
- 相机必须先做内参标定（`camera_info` 里的 fx/fy/cx/cy 和畸变），否则位姿(尤其距离)不准。

### 5.2 Python（`pupil-apriltags`）

```python
from pupil_apriltags import Detector
import cv2

# 相机标定结果(来自棋盘格标定)
fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0   # 替换为你的实测值

det = Detector(families="tag36h11", nthreads=2)
gray = cv2.imread("tag36_11_00000.png", cv2.IMREAD_GRAYSCALE)  # 真实使用时换成相机帧
dets = det.detect(
    gray,
    estimate_tag_pose=True,
    camera_params=(fx, fy, cx, cy),
    tag_size=0.119,            # ← 实测米数
)
for d in dets:
    print("id", d.tag_id, "hamming", d.hamming)
    print("pose_t (m):", d.pose_t.ravel())   # 平移: [x, y, z]
```

---

## 6. 如何确认这张图是对的（校验 / 复现）

任何人都可以独立验证本标签内容正确：

```bash
python gen_apriltag.py     # 重新生成 PNG
```

脚本逻辑：
- 取官方 `tag36h11.c` 中 `codedata[0] = 0x0d7e00984b`；
- 36 个数据位按 `bit_x/bit_y` 坐标放置；
- 位序按 `apriltag.c` 官方渲染函数：`code & (1 << (nbits-1-i))` 决定该格白/黑；
- 黑色外框 + 1 格白边 quiet zone。

可写回读脚本（解码 PNG 的 36 个数据位重拼码字）确认等于 `0x0d7e00984b`，即 ID 0。

---

## 7. 自定义（换尺寸 / 换 ID）

**换黑色方块尺寸**：改 `gen_print.py` 顶部一行后重跑：

```python
BLACK_SQUARE_MM = 100   # 例如想要 100mm，就填 100；150mm 就填 150
```

**换 ID**：把 `gen_apriltag.py` / `gen_print.py` 里的 `CODE` 换成对应码字（来自 `apriltag` 仓库 `tag36h11.c` 的 `codedata` 数组，下标 = ID）。前几个 ID 的码字供参考：

| ID | 码字 |
|---|---|
| 0 | `0x0d7e00984b` |
| 1 | `0x0da664ca7` |
| 2 | `0x0dc4a1c821` |
| 3 | `0x0e17b470e9` |
| 4 | `0x0ef91d01b1` |

需要其它 ID 的码字，可从官方 `tag36h11.c` 的 `codedata[]` 数组按行取（数组下标即为 ID）。

---

## 8. 摆放与几何建议（视觉泊车场景）

针对「小车自主泊入 30 cm × 30 cm 区域」这类任务：

- **标签**：竖直立于泊车区**远端正中**，中心高度对到相机镜头高度；支架底座 ≤1~2 cm，投影不得超出泊车区。
- **相机安装**：建议放在**车体中段**（非车头）。这样整车完全泊入时，相机距标签仍有 ~12.5~17.5 cm，标签全程不会超出画面。
- **停车目标**：相机↔标签 ≈ **15 cm**。此时标签占画面 ~57%，位姿精度最高，且整车基本居中泊入。
- **注意车头前置量**：若相机在中段，车头比相机多探出约半个车长，停车时车头与标签支架间要留 ≥2 cm 余量，避免碰撞。
- **参考 FOV 表**（按水平视场约 70° 估算，相机标称 80° 通常指对角线）：

  | 相机↔标签 | 12cm 标签占画面 | 评价 |
  |---|---|---|
  | 100 cm | ~9% | 远，仍可检测 |
  | 30 cm | ~29% | 良好 |
  | **15 cm** | **~57%** | **最佳** |
  | 12 cm | ~71% | 很好 |
  | 10 cm | ~86% | 偏满，需严格居中 |
  | ≤8 cm | >100% | 出画面，检测丢失 |

---

## 9. 许可声明

标签数据源自 `AprilRobotics/apriltag`（BSD-2-Clause）。本目录中的生成脚本与文档可自由使用与修改。分发时请保留本说明与第 3 节的来源链接。
