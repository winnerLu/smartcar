#!/usr/bin/env python3
"""STM32 底盘串口通信独立验证工具(不依赖 ROS)。

用途:在香橙派上快速确认与 STM32 的串口收发是否正确。
依赖:pip install pyserial

示例:
  # 只监听反馈帧,打印解析结果与统计(不发送任何运动指令)
  python3 serial_check.py --port /dev/ttyUSB0 --listen

  # 发送一次低速指令并观察反馈(默认前进 0.05 m/s 持续 2 秒后停)
  python3 serial_check.py --port /dev/ttyUSB0 --vx 0.05 --duration 2

  # 原地慢转,用于标定 wz 正负号(左转:angular.z>0)
  python3 serial_check.py --port /dev/ttyUSB0 --wz 0.3 --duration 3

安全提示:测试前请把小车架起,让轮子悬空,避免乱跑或撞击(手册 7.3)。
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print('缺少 pyserial,请先: pip install pyserial', file=sys.stderr)
    sys.exit(1)

# protocol.py 与本脚本同目录
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from protocol import build_ctrl_frame, FrameReader, FB_FRAME_LEN


def fmt_fb(fb):
    return (f'v=({fb.vx:+.3f},{fb.vy:+.3f}) m/s  '
            f'wz={fb.wz:+.4f} rad/s(raw={fb.raw_wz})  '
            f'acc=({fb.acc[0]:+.2f},{fb.acc[1]:+.2f},{fb.acc[2]:+.2f})  '
            f'gyro=({fb.gyro[0]:+.3f},{fb.gyro[1]:+.3f},{fb.gyro[2]:+.3f})  '
            f'bat={fb.voltage:.2f}V  stop={fb.flag_stop}')


def main():
    ap = argparse.ArgumentParser(description='STM32 底盘串口通信验证')
    ap.add_argument('--port', default='/dev/ttyUSB0', help='串口设备')
    ap.add_argument('--baud', type=int, default=115200, help='波特率')
    ap.add_argument('--listen', action='store_true',
                    help='只监听,不发送运动指令')
    ap.add_argument('--vx', type=float, default=0.0, help='线速度 m/s')
    ap.add_argument('--wz', type=float, default=0.0, help='角速度 rad/s')
    ap.add_argument('--cmd-wz-sign', type=int, default=1, choices=[1, -1],
                    help='角速度符号(实测后确定)')
    ap.add_argument('--rate', type=float, default=20.0, help='发送频率 Hz')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='发送指令持续秒数(0=一直,监听模式忽略)')
    ap.add_argument('--hex', action='store_true', help='额外打印原始帧十六进制')
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f'打开串口失败: {e}', file=sys.stderr)
        print('检查: 设备名对不对? 当前用户是否在 dialout 组? '
              '(sudo usermod -aG dialout $USER 后重新登录)', file=sys.stderr)
        sys.exit(1)

    print(f'已打开 {args.port} @ {args.baud}')
    if args.listen:
        print('监听模式:仅接收反馈帧,不发送指令。Ctrl+C 退出。\n')
    else:
        print(f'发送模式:vx={args.vx} m/s, wz={args.wz} rad/s, '
              f'{args.rate}Hz, 持续 {args.duration or "∞"} 秒。\n'
              f'⚠️ 确认小车已架空!Ctrl+C 立即停车退出。\n')

    reader = FrameReader()
    t0 = time.time()
    last_send = 0.0
    last_stat = 0.0
    send_interval = 1.0 / args.rate

    def send_stop():
        try:
            ser.write(build_ctrl_frame(0.0, 0.0, args.cmd_wz_sign))
        except Exception:
            pass

    try:
        while True:
            now = time.time()

            # 发送控制帧
            if not args.listen and (now - last_send) >= send_interval:
                over = args.duration and (now - t0) >= args.duration
                vx = 0.0 if over else args.vx
                wz = 0.0 if over else args.wz
                frame = build_ctrl_frame(vx, wz, args.cmd_wz_sign)
                ser.write(frame)
                last_send = now
                if args.hex and not over:
                    print('TX:', frame.hex(' '))

            # 接收并解析
            data = ser.read(FB_FRAME_LEN * 2)
            if data:
                for fb in reader.feed(data):
                    print(fmt_fb(fb))
                    if args.hex:
                        print('  RX:', fb.raw_frame.hex(' '))

            # 每秒打印统计
            if (now - last_stat) >= 1.0:
                print(f'  [统计] ok={reader.ok} bcc_err={reader.bcc_err} '
                      f'tail_err={reader.tail_err} resync={reader.resync}')
                last_stat = now
                if reader.ok == 0 and (now - t0) > 3:
                    print('  ⚠️ 3 秒未收到有效帧。检查: 接线/波特率/是否上电/'
                          'TX-RX 是否接反/设备名是否正确。')

            time.sleep(0.005)

    except KeyboardInterrupt:
        print('\n退出中,发送停车指令...')
        for _ in range(5):
            send_stop()
            time.sleep(0.02)
        ser.close()
        print(f'最终统计: ok={reader.ok} bcc_err={reader.bcc_err} '
              f'tail_err={reader.tail_err} resync={reader.resync}')


if __name__ == '__main__':
    main()
