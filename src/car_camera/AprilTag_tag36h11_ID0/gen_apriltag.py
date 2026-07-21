#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本地生成 AprilTag tag36h11 #0 的 PNG（无任何第三方依赖，纯标准库）。

权威来源：
- codebook 与 bit_x/bit_y 取自 AprilRobotics/apriltag 官方源码 tag36h11.c
- 渲染规则（位序：MSB 对应 i=0）取自 apriltag.c 的官方图像生成函数
  if (code & (1 << (nbits - i - 1))) cell(bit_x[i], bit_y[i]) = white

输出：10x10 格 = 1 格白边 quiet zone + 8x8 标签（1 格黑边 + 6x6 数据）。
"""
import os, struct, zlib

# ---- tag36h11 #0 的码字（codedata[0]，来自 tag36h11.c）----
CODE = 0x0d7e00984b
NBITS = 36

# 数据位在 8x8 标签格里的坐标（1..6），来自 tag36h11.c 的 bit_x/bit_y
BIT_X = [1,2,3,4,5,2,3,4,3,6,6,6,6,6,5,5,5,4,6,5,4,3,2,5,4,3,4,1,1,1,1,1,2,2,2,3]
BIT_Y = [1,1,1,1,1,2,2,2,3,1,2,3,4,5,2,3,4,3,6,6,6,6,6,5,5,5,4,6,5,4,3,2,5,4,3,4]

TOTAL_CELLS = 10      # total_width：含白边
WIDTH_AT_BORDER = 8   # 黑色外框外沿的格数（打印时量这个边长 = tag_size）
SCALE = 120           # 每格像素数，越大越清晰（120 -> 1200x1200）

def build_grid():
    # grid[row][col], 0=黑, 255=白；初始全黑
    g = [[0]*TOTAL_CELLS for _ in range(TOTAL_CELLS)]
    # 白边 quiet zone（最外圈）
    for k in range(TOTAL_CELLS):
        g[0][k] = g[TOTAL_CELLS-1][k] = 255
        g[k][0] = g[k][TOTAL_CELLS-1] = 255
    # 黑边框：标签格 1..8 的外圈，保持 0（已为黑）
    # 数据区：标签格 2..7，按码字填
    for i in range(NBITS):
        col = BIT_X[i] + 1   # bit_x 为列
        row = BIT_Y[i] + 1   # bit_y 为行
        if (CODE >> (NBITS - 1 - i)) & 1:   # MSB-first，与 apriltag.c 渲染一致
            g[row][col] = 255
    return g

def write_png_gray(path, pixels, w, h):
    """pixels: bytes, 长度 w*h, 行优先, 8-bit 灰度。"""
    def chunk(typ, data):
        return (struct.pack(">I", len(data)) + typ + data +
                struct.pack(">I", zlib.crc32(typ + data) & 0xffffffff))
    raw = bytearray()
    for y in range(h):
        raw.append(0)  # filter: None
        raw.extend(pixels[y*w:(y+1)*w])
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 9)))
        f.write(chunk(b"IEND", b""))

def main():
    grid = build_grid()
    n = TOTAL_CELLS * SCALE
    px = bytearray(n * n)
    for row in range(TOTAL_CELLS):
        for col in range(TOTAL_CELLS):
            v = grid[row][col]
            for dy in range(SCALE):
                base = ((row*SCALE + dy) * n + col*SCALE)
                px[base:base+SCALE] = bytes([v]) * SCALE
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tag36_11_00000.png")
    write_png_gray(out, px, n, n)
    # 打印结构核对
    black = sum(1 for r in grid for v in r if v == 0)
    white = sum(1 for r in grid for v in r if v == 255)
    print("written:", out)
    print("image size: %dx%d px" % (n, n))
    print("cells: %dx%d, black=%d white=%d" % (TOTAL_CELLS, TOTAL_CELLS, black, white))
    print("black-border outer edge = %d cells -> print this edge to your target cm" % WIDTH_AT_BORDER)

if __name__ == "__main__":
    main()
