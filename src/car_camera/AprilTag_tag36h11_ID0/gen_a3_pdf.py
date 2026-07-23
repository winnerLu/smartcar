#!/usr/bin/env python3
"""生成 A3 纸 297x300mm 停车板 PDF: 8 个 5cm 黑色方块完全贴边,四角+四边中点,外围黑色裁剪线。

关键点(贴边版):
  - 板子尺寸 297mm(宽) x 300mm(高): 宽度打满 A3, 高度居中。
  - 标签黑色方块 50mm 直接贴板边, 黑色方块外缘距板边 0mm。
  - 边缘标签外侧无白色 quiet zone, 靠板子下方的桌面/背景提供对比。
    建议把板子放在浅色(非黑)桌面上使用, 保证边缘标签检测率。
"""

import os, struct, zlib

# ---- A3 + 板子(mm) ----
A3_W, A3_H = 297, 420
BOARD_W_MM = 297   # 板子宽度(打满 A3 宽)
BOARD_H_MM = 300   # 板子高度
DPI = 300
PIX_PER_MM = DPI / 25.4

CANVAS_W = int(A3_W * PIX_PER_MM)
CANVAS_H = int(A3_H * PIX_PER_MM)

# 板子中心在 A3 正中(宽 297 = A3 宽, 无水平余量)
board_cx_mm = A3_W / 2
board_cy_mm = A3_H / 2
board_origin_x = board_cx_mm - BOARD_W_MM / 2   # = 0, 板子贴 A3 左右边
board_origin_y = board_cy_mm - BOARD_H_MM / 2   # = 60, 上下各留 60mm

# 标签黑色方块本身 50mm, tags_5cm/*.png 整张就是 50mm 黑色方块
TAG_BLACK_MM = 50
TAG_HALF = TAG_BLACK_MM / 2   # 25mm, 贴边放置时中心距板边 25mm

# 标签位置: 四角 + 四边中点, 黑色方块外缘贴板边(中心距板边 25mm)
# ID, 标签中心在板坐标系中的位置 (mm), 原点为板左上角
TAG_POSITIONS_BOARD = [
    (0, TAG_HALF,              TAG_HALF),              # 左上
    (1, BOARD_W_MM/2,          TAG_HALF),              # 上中
    (2, BOARD_W_MM-TAG_HALF,   TAG_HALF),              # 右上
    (3, BOARD_W_MM-TAG_HALF,   BOARD_H_MM/2),          # 右中
    (4, BOARD_W_MM-TAG_HALF,   BOARD_H_MM-TAG_HALF),   # 右下
    (5, BOARD_W_MM/2,          BOARD_H_MM-TAG_HALF),   # 下中
    (6, TAG_HALF,              BOARD_H_MM-TAG_HALF),   # 左下
    (7, TAG_HALF,              BOARD_H_MM/2),          # 左中
]

TAG_DIR = '/home/yangqingxi/rviz_ros2/smartcar/src/car_camera/AprilTag_tag36h11_ID0/tags_5cm'


def read_png(path):
    with open(path, 'rb') as f:
        data = f.read()
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    pos, w, h = 8, 0, 0
    idat_chunks = []
    while pos < len(data):
        length = struct.unpack('>I', data[pos:pos+4])[0]
        typ = data[pos+4:pos+8]
        if typ == b'IHDR':
            w = struct.unpack('>I', data[pos+8:pos+12])[0]
            h = struct.unpack('>I', data[pos+12:pos+16])[0]
        elif typ == b'IDAT':
            idat_chunks.append(data[pos+8:pos+8+length])
        elif typ == b'IEND':
            break
        pos += 12 + length
    raw = zlib.decompress(b''.join(idat_chunks))
    px = bytearray(w * h)
    row_size = 1 + w
    for y in range(h):
        px[y*w:(y+1)*w] = raw[y*row_size+1:(y+1)*row_size]
    return w, h, bytes(px)


def write_png(path, pixels, w, h):
    def chunk(typ, d):
        return struct.pack(">I", len(d)) + typ + d + struct.pack(">I", zlib.crc32(typ + d) & 0xffffffff)
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
    # 加载标签
    tags = {}
    for i in range(8):
        path = os.path.join(TAG_DIR, f'tag36h11_id{i:02d}.png')
        w, h, px = read_png(path)
        tags[i] = (w, h, px)
        print(f'Loaded tag {i}: {w}x{h}')

    tag_out_px = int(TAG_BLACK_MM * PIX_PER_MM)

    print(f'\nCanvas: {CANVAS_W}x{CANVAS_H}px @ {DPI}DPI ({A3_W}x{A3_H}mm)')
    print(f'Board: {BOARD_W_MM}x{BOARD_H_MM}mm, origin at ({board_origin_x:.1f},{board_origin_y:.1f})mm in A3')
    print(f'Tag black square: {TAG_BLACK_MM}mm -> {tag_out_px}px, 贴边放置(外缘距板边0mm)')

    # 白色画布
    canvas = bytearray([255]) * (CANVAS_W * CANVAS_H)

    # 放置标签(贴边)
    for id_val, bx_mm, by_mm in TAG_POSITIONS_BOARD:
        cx_px = int((board_origin_x + bx_mm) * PIX_PER_MM)
        cy_px = int((board_origin_y + by_mm) * PIX_PER_MM)

        tw, th, tpx = tags[id_val]
        scale = tag_out_px / tw
        scaled_w = int(tw * scale)
        scaled_h = int(th * scale)
        half_w = scaled_w // 2
        half_h = scaled_h // 2

        for sy in range(scaled_h):
            src_y = int(sy / scale)
            if src_y >= th:
                continue
            for sx in range(scaled_w):
                src_x = int(sx / scale)
                if src_x >= tw:
                    continue
                dx = cx_px - half_w + sx
                dy = cy_px - half_h + sy
                if 0 <= dx < CANVAS_W and 0 <= dy < CANVAS_H:
                    canvas[dy * CANVAS_W + dx] = tpx[src_y * tw + src_x]

        label = {0:'左上',1:'上中',2:'右上',3:'右中',4:'右下',5:'下中',6:'左下',7:'左中'}[id_val]
        print(f'  Tag {id_val}({label}) 板中心({bx_mm:.1f},{by_mm:.1f})mm')

    # 画黑色裁切线(3px粗)
    board_x0 = int(board_origin_x * PIX_PER_MM)
    board_y0 = int(board_origin_y * PIX_PER_MM)
    board_x1 = int((board_origin_x + BOARD_W_MM) * PIX_PER_MM)
    board_y1 = int((board_origin_y + BOARD_H_MM) * PIX_PER_MM)
    line_px = 3

    for t in range(line_px):
        # 上
        y = board_y0 + t
        if 0 <= y < CANVAS_H:
            for x in range(max(0, board_x0), min(CANVAS_W, board_x1)):
                canvas[y * CANVAS_W + x] = 0
        # 下
        y = board_y1 - 1 - t
        if 0 <= y < CANVAS_H:
            for x in range(max(0, board_x0), min(CANVAS_W, board_x1)):
                canvas[y * CANVAS_W + x] = 0
        # 左
        x = board_x0 + t
        if 0 <= x < CANVAS_W:
            for y in range(max(0, board_y0), min(CANVAS_H, board_y1)):
                canvas[y * CANVAS_W + x] = 0
        # 右
        x = board_x1 - 1 - t
        if 0 <= x < CANVAS_W:
            for y in range(max(0, board_y0), min(CANVAS_H, board_y1)):
                canvas[y * CANVAS_W + x] = 0

    # 保存 PNG
    out_dir = os.path.dirname(TAG_DIR)
    png_path = os.path.join(out_dir, 'parking_board_A3.png')
    write_png(png_path, canvas, CANVAS_W, CANVAS_H)

    # 转 PDF
    pdf_path = os.path.join(out_dir, 'parking_board_A3.pdf')
    try:
        import img2pdf
        with open(pdf_path, 'wb') as f:
            layout = img2pdf.get_layout_fun(pagesize=(A3_W * 72 / 25.4, A3_H * 72 / 25.4))
            f.write(img2pdf.convert(png_path, layout_fun=layout))
        print(f'\nPDF saved: {pdf_path}')
    except Exception as e:
        print(f'\nPDF failed: {e}')
        print(f'Use PNG instead: {png_path}')
        return

    print(f'PNG saved: {png_path}')
    print(f'\n=== 打印说明 ===')
    print(f'纸张: A3 ({A3_W}x{A3_H}mm)')
    print(f'打印: 实际大小 / 100%, 禁止缩放')
    print(f'板子: {BOARD_W_MM}x{BOARD_H_MM}mm, 沿黑色裁切线剪下')
    print(f'板宽 {BOARD_W_MM}mm = A3 宽, 上下各留 {(A3_H-BOARD_H_MM)/2:.0f}mm 白边')
    print(f'注意: 边缘标签外侧无白色 quiet zone, 需放在浅色(非黑)桌面上使用')
    print(f'标签黑色方块: {TAG_BLACK_MM}mm (务必实测确认)')


if __name__ == '__main__':
    main()
