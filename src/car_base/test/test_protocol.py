"""协议编解码单元测试。

用《总体方案设计 v1.2.1》6.5 节给出的确切示例值做断言。
无需硬件,WSL 上即可运行:  python3 -m pytest test/test_protocol.py -v
"""

import struct
import pytest

from car_base.protocol import (
    build_ctrl_frame, parse_feedback, bcc, FrameReader,
    FRAME_HEAD, FRAME_TAIL, CTRL_FRAME_LEN, FB_FRAME_LEN,
)


def make_fb_frame(vx_raw, vy_raw, vz_raw, ax, ay, az, gx, gy, gz, batt_mv):
    """按协议拼一个合法 24 字节反馈帧(自动算 BCC)。"""
    body = struct.pack('>BB hhh hhh hhh H',
                       FRAME_HEAD, 0,
                       vx_raw, vy_raw, vz_raw,
                       ax, ay, az,
                       gx, gy, gz,
                       batt_mv)
    assert len(body) == 22
    return body + bytes([bcc(body), FRAME_TAIL])


def test_ctrl_frame_structure():
    f = build_ctrl_frame(0.257, 0.0)
    assert len(f) == CTRL_FRAME_LEN
    assert f[0] == FRAME_HEAD
    assert f[-1] == FRAME_TAIL
    # BCC 覆盖字节 1..9(索引 0..8)
    assert f[9] == bcc(f[0:9])
    # vx = 0.257 m/s -> 257 = 0x0101 大端
    assert f[2] == 0x01 and f[3] == 0x01


def test_ctrl_frame_negative_vx():
    # -0.257 m/s -> -257 补码大端
    f = build_ctrl_frame(-0.257, 0.0)
    vx_raw = struct.unpack('>h', f[2:4])[0]
    assert vx_raw == -257


def test_ctrl_wz_sign():
    f_pos = build_ctrl_frame(0.0, 0.5, cmd_wz_sign=1)
    f_neg = build_ctrl_frame(0.0, 0.5, cmd_wz_sign=-1)
    wz_pos = struct.unpack('>h', f_pos[6:8])[0]
    wz_neg = struct.unpack('>h', f_neg[6:8])[0]
    assert wz_pos == 500
    assert wz_neg == -500


def test_parse_feedback_doc_example():
    # 文档 6.5: X速度 0x0101=257 -> 0.257 m/s
    #          Z加速度 0x4080=16512 -> /1672 = 9.8756 m/s^2
    #          电池 0x5838=22584 mV -> 22.584 V
    frame = make_fb_frame(
        vx_raw=0x0101, vy_raw=0, vz_raw=0,
        ax=0, ay=0, az=0x4080,
        gx=0, gy=0, gz=0,
        batt_mv=0x5838)
    fb = parse_feedback(frame)
    assert fb.vx == pytest.approx(0.257, abs=1e-6)
    assert fb.acc[2] == pytest.approx(9.8756, abs=1e-3)
    assert fb.voltage == pytest.approx(22.584, abs=1e-6)


def test_parse_feedback_negative():
    # 文档 6.5 负数示例: 0xFE96=-362 -> /1672=-0.2165 m/s^2
    #                    0xFFFB=-5 -> /3753=-0.00133 rad/s
    frame = make_fb_frame(
        vx_raw=0, vy_raw=0, vz_raw=0,
        ax=struct.unpack('>h', b'\xFE\x96')[0], ay=0, az=0,
        gx=0, gy=0, gz=struct.unpack('>h', b'\xFF\xFB')[0],
        batt_mv=12000)
    fb = parse_feedback(frame)
    assert fb.acc[0] == pytest.approx(-0.2165, abs=1e-3)
    assert fb.gyro[2] == pytest.approx(-0.00133, abs=1e-4)


def test_bcc_detects_corruption():
    frame = bytearray(make_fb_frame(1, 0, 0, 0, 0, 0, 0, 0, 0, 12000))
    frame[5] ^= 0xFF                    # 破坏一个数据字节
    with pytest.raises(ValueError, match='BCC'):
        parse_feedback(bytes(frame))


def test_frame_reader_resync_with_garbage():
    """流里混入垃圾字节 + 半帧,验证帧同步能恢复。"""
    good = make_fb_frame(0x0101, 0, 0, 0, 0, 0, 0, 0, 0, 12000)
    reader = FrameReader()
    # 前面垃圾 + 一整帧 + 下一帧的前半段
    stream = b'\x00\xFF\xAB' + good + good[:10]
    out = reader.feed(stream)
    assert len(out) == 1
    assert out[0].vx == pytest.approx(0.257, abs=1e-6)
    # 再喂入后半段,应拼出第二帧
    out2 = reader.feed(good[10:])
    assert len(out2) == 1
    assert reader.ok == 2


def test_roundtrip_wz_sign():
    """发送套 cmd_wz_sign,接收套 odom_wz_sign,同号应还原。"""
    sign = -1
    f = build_ctrl_frame(0.0, 0.4, cmd_wz_sign=sign)
    raw = struct.unpack('>h', f[6:8])[0]      # = -400
    # 接收端用同样符号还原
    wz_ros = sign * raw / 1000.0
    assert wz_ros == pytest.approx(0.4, abs=1e-6)
