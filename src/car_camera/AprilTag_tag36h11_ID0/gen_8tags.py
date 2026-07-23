#!/usr/bin/env python3
"""批量生成 AprilTag tag36h11 ID 0-7 的 PNG(5cm黑色方块)"""

import os, struct, zlib

CODES = {
    0: 0x0d7e00984b, 1: 0x0da664ca7, 2: 0x0dc4a1c821, 3: 0x0e17b470e9,
    4: 0x0ef91d01b1, 5: 0x0f429cdd73, 6: 0x005da29225, 7: 0x01106cba43,
}
NBITS = 36
BIT_X = [1,2,3,4,5,2,3,4,3,6,6,6,6,6,5,5,5,4,6,5,4,3,2,5,4,3,4,1,1,1,1,1,2,2,2,3]
BIT_Y = [1,1,1,1,1,2,2,2,3,1,2,3,4,5,2,3,4,3,6,6,6,6,6,5,5,5,4,6,5,4,3,2,5,4,3,4]
TOTAL = 10
SCALE = 60  # 600px total, black square = 480px

def build_grid(code):
    g = [[0]*TOTAL for _ in range(TOTAL)]
    for k in range(TOTAL):
        g[0][k] = g[TOTAL-1][k] = 255
        g[k][0] = g[k][TOTAL-1] = 255
    for i in range(NBITS):
        if (code >> (NBITS - 1 - i)) & 1:
            g[BIT_Y[i] + 1][BIT_X[i] + 1] = 255
    return g

def write_png(path, pixels, w, h):
    def chunk(typ, data):
        return (struct.pack(">I", len(data)) + typ + data +
                struct.pack(">I", zlib.crc32(typ + data) & 0xffffffff))
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw.extend(pixels[y*w:(y+1)*w])
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(bytes(raw), 9)))
        f.write(chunk(b"IEND", b""))

def main():
    outdir = '/home/yangqingxi/rviz_ros2/smartcar/src/car_camera/AprilTag_tag36h11_ID0/tags_5cm'
    os.makedirs(outdir, exist_ok=True)

    for id_val in range(8):
        code = CODES[id_val]
        grid = build_grid(code)
        n = TOTAL * SCALE
        px = bytearray(n * n)
        for row in range(TOTAL):
            for col in range(TOTAL):
                v = grid[row][col]
                for dy in range(SCALE):
                    base = ((row*SCALE + dy) * n + col*SCALE)
                    px[base:base+SCALE] = bytes([v]) * SCALE
        path = os.path.join(outdir, f'tag36h11_id{id_val:02d}.png')
        write_png(path, px, n, n)
        print(f'ID {id_val}: {path} ({n}x{n}px)')

    print(f'\nDone: {len(CODES)} tags in {outdir}')

if __name__ == '__main__':
    main()
