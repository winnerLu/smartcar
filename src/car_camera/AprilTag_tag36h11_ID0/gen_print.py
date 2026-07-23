# -*- coding: utf-8 -*-
"""
生成 A4 打印版 AprilTag tag36h11 #0（PDF）。
- 黑色方块边长 = BLACK_SQUARE_MM（默认 120 mm，即 tag_size=0.12）
- 含 1 格白边 quiet zone（打印在白纸上自动隐形）
- 标签下方有与黑色方块等宽的标尺，打印后用尺子核对是否 1:1
纯标准库，无需第三方依赖。
"""
import os, zlib

# ============ 可调参数 ============
BLACK_SQUARE_MM = 120          # 黑色方块边长（= 你要填进 apriltag_ros 的 tag_size，单位 mm）
SCALE_PX_PER_CELL = 200        # 每格像素数，越大越清晰（200 -> 2000x2000，约 338 DPI @15cm）
OUT_PDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tag36h11_0_A4_120mm.pdf")
# ==================================

# tag36h11 #0：码字 + 数据位坐标（来自 AprilRobotics/apriltag 官方源码）
CODE = 0x0d7e00984b
NBITS = 36
BIT_X = [1,2,3,4,5,2,3,4,3,6,6,6,6,6,5,5,5,4,6,5,4,3,2,5,4,3,4,1,1,1,1,1,2,2,2,3]
BIT_Y = [1,1,1,1,1,2,2,2,3,1,2,3,4,5,2,3,4,3,6,6,6,6,6,5,5,5,4,6,5,4,3,2,5,4,3,4]
TOTAL = 10
WIDTH_AT_BORDER = 8

def build_grid():
    g = [[0]*TOTAL for _ in range(TOTAL)]
    for k in range(TOTAL):
        g[0][k] = g[TOTAL-1][k] = 255
        g[k][0] = g[k][TOTAL-1] = 255
    for i in range(NBITS):
        if (CODE >> (NBITS - 1 - i)) & 1:
            g[BIT_Y[i]+1][BIT_X[i]+1] = 255
    return g

def render_pixels(grid):
    s = SCALE_PX_PER_CELL
    n = TOTAL * s
    px = bytearray(n * n)
    for r in range(TOTAL):
        for c in range(TOTAL):
            v = grid[r][c]
            fill = bytes([v]) * s
            for dy in range(s):
                base = ((r*s + dy) * n) + c*s
                px[base:base+s] = fill
    return bytes(px), n

MM = 72.0 / 25.4
A4W, A4H = 210*MM, 297*MM
CELL_MM = BLACK_SQUARE_MM / WIDTH_AT_BORDER
IMG_MM = TOTAL * CELL_MM
IMG_P = IMG_MM * MM
X0 = (A4W - IMG_P) / 2
Y0 = 330.0
BS_LEFT = X0 + CELL_MM*MM
BS_W = BLACK_SQUARE_MM * MM
BS_RIGHT = BS_LEFT + BS_W
RULER_Y = Y0 - 10
TICK_BOT = RULER_Y - 9
LBL_Y = RULER_Y - 21

def esc(s):
    return s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")

def text(x, y, s, size=11, font="F1"):
    return f"BT /{font} {size} Tf 1 0 0 1 {x:.3f} {y:.3f} Tm ({esc(s)}) Tj ET"

def build_pdf(px, npx):
    img_stream = zlib.compress(px, 9)
    cs = []
    cs.append("q")
    cs.append(f"{IMG_P:.4f} 0 0 {IMG_P:.4f} {X0:.4f} {Y0:.4f} cm /Im1 Do")
    cs.append("Q")
    cs.append(f"{A4W/2-120:.3f} {A4H-46:.3f} m {A4W/2+120:.3f} {A4H-46:.3f} l S")  # title underline (optional)
    cs.append("0.7 w")
    # 标尺主线
    cs.append(f"{BS_LEFT:.3f} {RULER_Y:.3f} m {BS_RIGHT:.3f} {RULER_Y:.3f} l S")
    # 每 10mm 一格刻度
    for k in range(0, int(BLACK_SQUARE_MM//10) + 1):
        x = BS_LEFT + k*10*MM
        cs.append(f"{x:.3f} {RULER_Y:.3f} m {x:.3f} {TICK_BOT:.3f} l S")
    content = "\n".join(cs).encode("latin-1")
    # 文字
    content += b"\n"
    content += text(A4W/2-110, A4H-40, "AprilTag  tag36h11   ID 0", 18).encode("latin-1") + b"\n"
    content += text(A4W/2-150, A4H-58, "Print at 100% / Actual size. Do NOT use 'Fit to Page'.", 11).encode("latin-1") + b"\n"
    # 标尺数字 0 / 4 / 8 / 12 cm
    for cmv in [0, 4, 8, int(BLACK_SQUARE_MM//10)]:
        x = BS_LEFT + cmv*10*MM
        content += text(x-6 if cmv>0 else x-2, LBL_Y, f"{cmv} cm", 9).encode("latin-1") + b"\n"
    # 底部说明
    content += text(60, 250, f"Black square edge = {BLACK_SQUARE_MM} mm  (= tag_size {BLACK_SQUARE_MM/1000.0:.3f} m).", 11).encode("latin-1") + b"\n"
    content += text(60, 235, "After printing, measure the black square edge with a ruler.", 11).encode("latin-1") + b"\n"
    content += text(60, 220, "If the ruler below is NOT true to size, reprint with scaling disabled.", 11).encode("latin-1") + b"\n"
    content += text(60, 205, "e.g. if it measures 119 mm, set  tag_size: 0.119  in apriltag_ros.", 11).encode("latin-1") + b"\n"

    objects = {}
    objects[1] = b"<< /Type /Catalog /Pages 2 0 R >>"
    objects[2] = b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>"
    objects[3] = (b"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] "
                  b"/Resources << /Font << /F1 5 0 R >> /XObject << /Im1 6 0 R >> /ProcSet [/PDF /Text /ImageC] >> "
                  b"/Contents 4 0 R >>") % (A4W, A4H)
    objects[4] = b"<< /Length %d >>\nstream\n" % len(content) + content + b"\nendstream"
    objects[5] = b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"
    objects[6] = (b"<< /Type /XObject /Subtype /Image /Width %d /Height %d "
                  b"/ColorSpace /DeviceGray /BitsPerComponent 8 "
                  b"/Filter /FlateDecode /Length %d >>\nstream\n" % (npx, npx, len(img_stream))
                  + img_stream + b"\nendstream")

    header = b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n"
    body = bytearray()
    offsets = {}
    for num in range(1, 7):
        offsets[num] = len(header) + len(body)
        body += b"%d 0 obj\n" % num + objects[num] + b"\nendobj\n"
    xref_off = len(header) + len(body)
    nobj = 7
    xref = bytearray()
    xref += b"xref\n0 %d\n" % nobj
    xref += b"0000000000 65535 f \r\n"
    for num in range(1, 7):
        xref += b"%010d 00000 n \r\n" % offsets[num]
    xref += (b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF" % (nobj, xref_off))
    with open(OUT_PDF, "wb") as f:
        f.write(header)
        f.write(body)
        f.write(xref)
    return len(header) + len(body) + len(xref)

grid = build_grid()
px, npx = render_pixels(grid)
sz = build_pdf(px, npx)
print("written:", OUT_PDF)
print("page: A4 (%.0f x %.0f pt); black square = %d mm; full square = %.0f mm"
      % (A4W, A4H, BLACK_SQUARE_MM, IMG_MM))
print("image: %dx%d px (~%.0f DPI); pdf size: %.1f KB" % (npx, npx, IMG_MM/25.4 and npx/(IMG_MM/25.4), sz/1024))
