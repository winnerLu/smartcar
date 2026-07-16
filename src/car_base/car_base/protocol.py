"""STM32 底盘串口协议编解码。

依据《总体方案设计 v1.2.1》第 6 章 / 小车手册第 6 章:
- 控制帧 (Orange Pi -> STM32): 11 字节
- 反馈帧 (STM32 -> Orange Pi): 24 字节
- 多字节字段为 int16 大端(网络字节序),补码表示
- BCC 校验 = 帧内数据字节逐字节异或(不含校验位与帧尾)

本模块不依赖 ROS,可独立用于串口通信验证。
"""

import struct
from dataclasses import dataclass

FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D

CTRL_FRAME_LEN = 11
FB_FRAME_LEN = 24

# IMU 原始计数 -> 物理量的换算系数(手册 6.2 明确值)
# 加速度计量程 ±2G: 32768 / 19.6 = 1672  -> m/s^2
# 角速度计量程 ±500°/s: 例 4/3753 -> rad/s
ACC_COUNT_PER_MS2 = 1672.0
GYRO_COUNT_PER_RADPS = 3753.0


def bcc(data: bytes) -> int:
    """逐字节异或校验。"""
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def build_ctrl_frame(vx: float, wz: float,
                     cmd_wz_sign: int = 1,
                     flag_stop: int = 0,
                     reserved: int = 0) -> bytes:
    """构造 11 字节控制帧。

    vx: 线速度 m/s;  wz: 角速度 rad/s(差速车 vy 恒为 0)。
    cmd_wz_sign: 角速度符号参数,实测后固定(手册对 Z 正方向有矛盾)。
    """
    raw_vx = int(round(vx * 1000.0))
    raw_vy = 0
    raw_wz = int(round(cmd_wz_sign * wz * 1000.0))

    # 大端 int16,越界则饱和
    def clamp16(v):
        return max(-32768, min(32767, v))

    body = struct.pack('>BBhhhB',
                       FRAME_HEAD,
                       flag_stop & 0xFF,
                       clamp16(raw_vx),
                       clamp16(raw_vy),
                       clamp16(raw_wz),
                       reserved & 0xFF)   # 字节 1..9
    frame = body + bytes([bcc(body), FRAME_TAIL])
    assert len(frame) == CTRL_FRAME_LEN
    return frame


@dataclass
class Feedback:
    """解析后的一帧反馈数据(已换算物理单位)。"""
    flag_stop: int
    vx: float          # m/s
    vy: float          # m/s
    wz: float          # rad/s (未套 odom_wz_sign,由上层套)
    acc: tuple         # (ax, ay, az) m/s^2
    gyro: tuple        # (gx, gy, gz) rad/s
    voltage: float     # V
    # 原始计数,便于调试/标定
    raw_wz: int = 0
    raw_frame: bytes = b''   # 原始 24 字节,便于 hex 调试


def parse_feedback(frame: bytes) -> Feedback:
    """解析 24 字节反馈帧。校验失败或长度不符则抛 ValueError。"""
    if len(frame) != FB_FRAME_LEN:
        raise ValueError(f'反馈帧长度应为 {FB_FRAME_LEN},实际 {len(frame)}')
    if frame[0] != FRAME_HEAD:
        raise ValueError(f'帧头错误: 0x{frame[0]:02X}')
    if frame[FB_FRAME_LEN - 1] != FRAME_TAIL:
        raise ValueError(f'帧尾错误: 0x{frame[FB_FRAME_LEN - 1]:02X}')
    calc = bcc(frame[0:22])          # 字节 1..22
    if calc != frame[22]:
        raise ValueError(f'BCC 校验失败: 计算 0x{calc:02X} != 收到 0x{frame[22]:02X}')

    # 字节 3..20 为 9 个 int16 大端: vx vy vz accX accY accZ gyroX gyroY gyroZ
    vx_raw, vy_raw, vz_raw, ax_c, ay_c, az_c, gx_c, gy_c, gz_c = \
        struct.unpack('>hhhhhhhhh', frame[2:20])
    voltage_mv = struct.unpack('>H', frame[20:22])[0]

    return Feedback(
        flag_stop=frame[1],
        vx=vx_raw / 1000.0,
        vy=vy_raw / 1000.0,
        wz=vz_raw / 1000.0,
        acc=(ax_c / ACC_COUNT_PER_MS2,
             ay_c / ACC_COUNT_PER_MS2,
             az_c / ACC_COUNT_PER_MS2),
        gyro=(gx_c / GYRO_COUNT_PER_RADPS,
              gy_c / GYRO_COUNT_PER_RADPS,
              gz_c / GYRO_COUNT_PER_RADPS),
        voltage=voltage_mv / 1000.0,
        raw_wz=vz_raw,
        raw_frame=bytes(frame),
    )


class FrameReader:
    """字节流帧同步器。

    串口 read() 不保证一次返回整帧,本类按帧头累积、校验、重同步。
    用法: reader.feed(bytes) -> 返回本次解析出的 Feedback 列表。
    """

    def __init__(self):
        self._buf = bytearray()
        # 诊断计数
        self.ok = 0
        self.bcc_err = 0
        self.tail_err = 0
        self.resync = 0

    def feed(self, data: bytes):
        out = []
        self._buf.extend(data)
        while True:
            # 找帧头
            i = self._buf.find(FRAME_HEAD)
            if i < 0:
                self._buf.clear()
                break
            if i > 0:
                del self._buf[:i]        # 丢弃帧头前的垃圾
                self.resync += 1
            if len(self._buf) < FB_FRAME_LEN:
                break                     # 数据不足一帧,等下次
            frame = bytes(self._buf[:FB_FRAME_LEN])
            try:
                fb = parse_feedback(frame)
                out.append(fb)
                self.ok += 1
                del self._buf[:FB_FRAME_LEN]
            except ValueError as e:
                msg = str(e)
                if 'BCC' in msg:
                    self.bcc_err += 1
                elif '帧尾' in msg:
                    self.tail_err += 1
                # 帧头后单字节跳过,重新同步
                del self._buf[0]
                self.resync += 1
        return out
