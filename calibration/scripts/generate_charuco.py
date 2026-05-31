#!/usr/bin/env python3
"""Generate a ChArUco board printable PDF (exact A4 scale, mm-accurate).

Output:
  boards/charuco_5x7_30mm_22mm.png  (high-res, 1200 DPI)
  boards/charuco_5x7_30mm_22mm.pdf  (A4 portrait, 1:1 print)

After printing, MEASURE one square with a ruler to verify 30.0 mm.
If a printer scales, recompute size and update the calibration script.
"""
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

# ---- Board spec ----------------------------------------------------------
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LEN_MM = 40.0
MARKER_LEN_MM = 30.0
DICTIONARY = cv2.aruco.DICT_5X5_50

# ---- Print spec ----------------------------------------------------------
PRINT_DPI = 600                 # final PDF density
A4_W_MM, A4_H_MM = 210.0, 297.0 # portrait
MARGIN_MM = 5.0                 # white margin around board (board fills A4)

OUT_DIR = Path(__file__).resolve().parent.parent / "boards"
OUT_DIR.mkdir(parents=True, exist_ok=True)
STEM = f"charuco_{SQUARES_X}x{SQUARES_Y}_{int(SQUARE_LEN_MM)}mm_{int(MARKER_LEN_MM)}mm"


def mm_to_px(mm, dpi):
    return int(round(mm * dpi / 25.4))


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LEN_MM / 1000.0,
        MARKER_LEN_MM / 1000.0,
        dictionary,
    )

    board_w_mm = SQUARES_X * SQUARE_LEN_MM
    board_h_mm = SQUARES_Y * SQUARE_LEN_MM
    board_w_px = mm_to_px(board_w_mm, PRINT_DPI)
    board_h_px = mm_to_px(board_h_mm, PRINT_DPI)
    board_img = board.generateImage((board_w_px, board_h_px), marginSize=0, borderBits=1)

    page_w_px = mm_to_px(A4_W_MM, PRINT_DPI)
    page_h_px = mm_to_px(A4_H_MM, PRINT_DPI)
    page = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)

    off_x = mm_to_px(MARGIN_MM, PRINT_DPI)
    off_y = mm_to_px(MARGIN_MM, PRINT_DPI)
    page[off_y:off_y + board_h_px, off_x:off_x + board_w_px] = board_img

    # 1 mm ruler tick row below the board for printer-scale verification
    tick_top = off_y + board_h_px + mm_to_px(6, PRINT_DPI)
    tick_bot = tick_top + mm_to_px(4, PRINT_DPI)
    for i in range(0, int(board_w_mm) + 1):
        x = off_x + mm_to_px(i, PRINT_DPI)
        h = mm_to_px(4 if i % 10 == 0 else (2.5 if i % 5 == 0 else 1.5), PRINT_DPI)
        page[tick_top:tick_top + h, x] = 0

    # Caption (bottom)
    caption = (
        f"ChArUco {SQUARES_X}x{SQUARES_Y}  "
        f"square={SQUARE_LEN_MM:.1f}mm  marker={MARKER_LEN_MM:.1f}mm  "
        f"DICT_5X5_50  Print at 100% scale"
    )
    cv2.putText(
        page, caption,
        (off_x, tick_bot + mm_to_px(10, PRINT_DPI)),
        cv2.FONT_HERSHEY_SIMPLEX, 1.4, 0, 2, cv2.LINE_AA,
    )

    png_path = OUT_DIR / f"{STEM}.png"
    pdf_path = OUT_DIR / f"{STEM}.pdf"
    cv2.imwrite(str(png_path), page)

    pil = Image.fromarray(page)
    pil.save(pdf_path, "PDF", resolution=PRINT_DPI)

    print(f"Board: {SQUARES_X}x{SQUARES_Y}, square {SQUARE_LEN_MM} mm, marker {MARKER_LEN_MM} mm")
    print(f"Outer size: {board_w_mm:.1f} x {board_h_mm:.1f} mm")
    print(f"PNG: {png_path}")
    print(f"PDF: {pdf_path}  ({A4_W_MM}x{A4_H_MM} mm @ {PRINT_DPI} DPI)")


if __name__ == "__main__":
    main()
