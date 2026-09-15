#!/usr/bin/env python3
"""
plot_constellation.py - Pure Standard Library 802.11 Constellation Visualizer
Reads IQ constellation dumps from quadrf-wifi and outputs crisp, dark-mode PNGs.
Zero third-party dependencies: uses built-in zlib and struct.
"""

import math
import os
import struct
import sys
import zlib

FONT_5X7 = {
    ' ': [0, 0, 0, 0, 0],
    '0': [0x3E, 0x51, 0x49, 0x45, 0x3E],
    '1': [0x00, 0x42, 0x7F, 0x40, 0x00],
    '2': [0x42, 0x61, 0x51, 0x49, 0x46],
    '3': [0x21, 0x41, 0x45, 0x4B, 0x31],
    '4': [0x18, 0x14, 0x12, 0x7F, 0x10],
    '5': [0x27, 0x45, 0x45, 0x45, 0x39],
    '6': [0x3C, 0x4A, 0x49, 0x49, 0x30],
    '7': [0x01, 0x71, 0x09, 0x05, 0x03],
    '8': [0x36, 0x49, 0x49, 0x49, 0x36],
    '9': [0x06, 0x49, 0x49, 0x29, 0x1E],
    'A': [0x7E, 0x11, 0x11, 0x11, 0x7E],
    'B': [0x7F, 0x49, 0x49, 0x49, 0x36],
    'C': [0x3E, 0x41, 0x41, 0x41, 0x22],
    'D': [0x7F, 0x41, 0x41, 0x22, 0x1C],
    'E': [0x7F, 0x49, 0x49, 0x49, 0x41],
    'F': [0x7F, 0x09, 0x09, 0x09, 0x01],
    'G': [0x3E, 0x41, 0x49, 0x49, 0x7A],
    'H': [0x7F, 0x08, 0x08, 0x08, 0x7F],
    'I': [0x00, 0x41, 0x7F, 0x41, 0x00],
    'K': [0x7F, 0x08, 0x14, 0x22, 0x41],
    'L': [0x7F, 0x40, 0x40, 0x40, 0x40],
    'M': [0x7F, 0x02, 0x0C, 0x02, 0x7F],
    'N': [0x7F, 0x04, 0x08, 0x10, 0x7F],
    'O': [0x3E, 0x41, 0x41, 0x41, 0x3E],
    'P': [0x7F, 0x09, 0x09, 0x09, 0x06],
    'Q': [0x3E, 0x41, 0x51, 0x21, 0x5E],
    'R': [0x7F, 0x09, 0x19, 0x29, 0x46],
    'S': [0x46, 0x49, 0x49, 0x49, 0x31],
    'T': [0x01, 0x01, 0x7F, 0x01, 0x01],
    'U': [0x3F, 0x40, 0x40, 0x40, 0x3F],
    'V': [0x1F, 0x20, 0x40, 0x20, 0x1F],
    'W': [0x3F, 0x40, 0x38, 0x40, 0x3F],
    'X': [0x63, 0x14, 0x08, 0x14, 0x63],
    'Y': [0x07, 0x08, 0x70, 0x08, 0x07],
    ':': [0x00, 0x36, 0x36, 0x00, 0x00],
    '.': [0x00, 0x60, 0x60, 0x00, 0x00],
    '-': [0x08, 0x08, 0x08, 0x08, 0x08],
    '+': [0x08, 0x08, 0x3E, 0x08, 0x08],
    '(': [0x00, 0x3E, 0x41, 0x00, 0x00],
    ')': [0x00, 0x41, 0x3E, 0x00, 0x00],
    '%': [0x23, 0x13, 0x08, 0x64, 0x62],
    '/': [0x20, 0x10, 0x08, 0x04, 0x02],
}


def write_png(filename, width, height, rgb_bytes):
    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF)

    raw = b''.join(b'\x00' + rgb_bytes[y * width * 3 : (y + 1) * width * 3] for y in range(height))
    png = (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
        + chunk(b'IDAT', zlib.compress(raw, 9))
        + chunk(b'IEND', b'')
    )
    with open(filename, 'wb') as f:
        f.write(png)


def draw_text(pixels, width, height, x, y, text, r, g, b):
    text = text.upper()
    cur_x = x
    for ch in text:
        bitmap = FONT_5X7.get(ch, FONT_5X7[' '])
        for col_idx, col in enumerate(bitmap):
            px = cur_x + col_idx
            if 0 <= px < width:
                for row_idx in range(7):
                    if (col >> row_idx) & 1:
                        py = y + row_idx
                        if 0 <= py < height:
                            idx = (py * width + px) * 3
                            pixels[idx] = r
                            pixels[idx + 1] = g
                            pixels[idx + 2] = b
        cur_x += 7


def render_constellation(points, out_path, title='802.11 Constellation', snr_db=None, evm_db=None, mod='BPSK', width=384, height=384):
    pixels = bytearray(width * height * 3)

    # 1. Dark Slate Background
    bg_r, bg_g, bg_b = 15, 17, 26
    for i in range(0, len(pixels), 3):
        pixels[i] = bg_r
        pixels[i + 1] = bg_g
        pixels[i + 2] = bg_b

    cx, cy = width // 2, height // 2
    scale = (width * 0.38) / 1.5  # +/-1.5 maps to 38% radius

    # 2. Draw Subtle Grid Axes
    grid_r, grid_g, grid_b = 35, 40, 58
    for x in range(width):
        idx = (cy * width + x) * 3
        pixels[idx] = grid_r
        pixels[idx + 1] = grid_g
        pixels[idx + 2] = grid_b
    for y in range(height):
        idx = (y * width + cx) * 3
        pixels[idx] = grid_r
        pixels[idx + 1] = grid_g
        pixels[idx + 2] = grid_b

    # Unit circle outline (dashed)
    r_unit = int(scale * 1.0)
    for deg in range(0, 360, 3):
        rad = math.radians(deg)
        px = int(cx + r_unit * math.cos(rad))
        py = int(cy - r_unit * math.sin(rad))
        if 0 <= px < width and 0 <= py < height:
            idx = (py * width + px) * 3
            pixels[idx] = 28
            pixels[idx + 1] = 32
            pixels[idx + 2] = 48

    # 3. Draw Ideal Constellation Points
    if '64' in mod or '64QAM' in mod or mod in ('48', '54'):
        levels = [-7, -5, -3, -1, 1, 3, 5, 7]
        norm = 1.0 / math.sqrt(42.0)
        ideal_pts = [(l1 * norm, l2 * norm) for l1 in levels for l2 in levels]
    elif '16' in mod or '16QAM' in mod or mod in ('24', '36'):
        levels = [-3, -1, 1, 3]
        norm = 1.0 / math.sqrt(10.0)
        ideal_pts = [(l1 * norm, l2 * norm) for l1 in levels for l2 in levels]
    elif 'QPSK' in mod or mod in ('12', '18'):
        norm = 1.0 / math.sqrt(2.0)
        ideal_pts = [(-norm, -norm), (-norm, norm), (norm, -norm), (norm, norm)]
    else:  # BPSK
        ideal_pts = [(-1.0, 0.0), (1.0, 0.0)]

    for ix, iy in ideal_pts:
        px = int(cx + ix * scale)
        py = int(cy - iy * scale)
        # Small 3x3 gold cross
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if (dx == 0 or dy == 0) and 0 <= px + dx < width and 0 <= py + dy < height:
                    idx = ((py + dy) * width + (px + dx)) * 3
                    pixels[idx] = 255
                    pixels[idx + 1] = 195
                    pixels[idx + 2] = 0

    # 4. Adaptive Gaussian KDE Density Map
    grid = [0.0] * (width * height)
    sigma = 3.0
    radius = 5
    kernel = []
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            d2 = dx * dx + dy * dy
            if d2 <= radius * radius:
                w = math.exp(-d2 / (2 * sigma * sigma))
                kernel.append((dx, dy, w))

    for x_val, y_val in points:
        px = int(cx + x_val * scale)
        py = int(cy - y_val * scale)
        for dx, dy, w in kernel:
            nx, ny = px + dx, py + dy
            if 0 <= nx < width and 0 <= ny < height:
                grid[ny * width + nx] += w

    max_d = max(grid) if grid else 1.0

    # Render smooth cyan / teal / glow heatmap
    for y in range(height):
        for x in range(width):
            d = grid[y * width + x]
            if d > 0.10:
                t = min(1.0, (d / (max_d * 0.70)) ** 1.3)
                if t < 0.25:
                    f = t / 0.25
                    r = int(bg_r + f * 30)
                    g = int(bg_g + f * 90)
                    b = int(bg_b + f * 170)
                elif t < 0.65:
                    f = (t - 0.25) / 0.40
                    r = int(30 + f * 90)
                    g = int(90 + f * 130)
                    b = int(170 + f * 70)
                else:
                    f = (t - 0.65) / 0.35
                    r = int(120 + f * 135)
                    g = int(220 + f * 35)
                    b = int(240 + f * 15)
                idx = (y * width + x) * 3
                pixels[idx] = min(255, r)
                pixels[idx + 1] = min(255, g)
                pixels[idx + 2] = min(255, b)

    # Crisp individual point markers
    for x_val, y_val in points:
        px = int(cx + x_val * scale)
        py = int(cy - y_val * scale)
        if 0 <= px < width and 0 <= py < height:
            idx = (py * width + px) * 3
            pixels[idx] = min(255, pixels[idx] + 70)
            pixels[idx + 1] = min(255, pixels[idx + 1] + 110)
            pixels[idx + 2] = min(255, pixels[idx + 2] + 140)

    # 5. Header / Badges
    draw_text(pixels, width, height, 14, 14, title, 240, 240, 255)
    badge = f"{mod}  N={len(points)}"
    draw_text(pixels, width, height, 14, 26, badge, 0, 230, 180)

    status_line = ""
    if snr_db is not None:
        status_line += f"SNR:{snr_db:.1f}DB  "
    if evm_db is not None:
        status_line += f"EVM:{evm_db:.1f}DB"
    if status_line:
        draw_text(pixels, width, height, 14, height - 20, status_line, 255, 200, 80)

    write_png(out_path, width, height, bytes(pixels))
    print(f"[+] Rendered constellation -> {out_path} ({len(points)} symbols)")


def main():
    if len(sys.argv) < 2:
        print("Usage: plot_constellation.py <dump_file.txt> [output.png]")
        sys.exit(1)

    in_file = sys.argv[1]
    out_file = sys.argv[2] if len(sys.argv) > 2 else "constellation.png"

    frames = []
    current_frame = None

    with open(in_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if current_frame and current_frame["points"]:
                    frames.append(current_frame)
                current_frame = {"points": [], "rate": "6", "snr": None, "evm": None, "fcs_ok": True}
                for p in line.split():
                    if p.startswith("rate="):
                        current_frame["rate"] = p.split("=")[1]
                    elif p.startswith("snr="):
                        try: current_frame["snr"] = float(p.split("=")[1])
                        except ValueError: pass
                    elif p.startswith("evm="):
                        try: current_frame["evm"] = float(p.split("=")[1])
                        except ValueError: pass
                    elif p == "fcs=FAIL":
                        current_frame["fcs_ok"] = False
                    elif p == "fcs=OK":
                        current_frame["fcs_ok"] = True
                continue
            coords = line.split()
            if len(coords) >= 2 and current_frame is not None:
                try:
                    current_frame["points"].append((float(coords[0]), float(coords[1])))
                except ValueError:
                    pass
    if current_frame and current_frame["points"]:
        frames.append(current_frame)

    # Prioritize FCS=OK frames if available
    ok_frames = [fr for fr in frames if fr["fcs_ok"]]
    selected = ok_frames if ok_frames else frames

    if not selected:
        print("[-] No valid frames found in input file.")
        sys.exit(1)

    points = []
    snrs = []
    evms = []
    mod = selected[0]["rate"]
    for fr in selected:
        points.extend(fr["points"])
        if fr["snr"] is not None: snrs.append(fr["snr"])
        if fr["evm"] is not None: evms.append(fr["evm"])

    snr = sum(snrs) / len(snrs) if snrs else None
    evm = sum(evms) / len(evms) if evms else None

    render_constellation(points, out_file, title="802.11 Over-the-Air IQ", snr_db=snr, evm_db=evm, mod=mod)


if __name__ == "__main__":
    main()
