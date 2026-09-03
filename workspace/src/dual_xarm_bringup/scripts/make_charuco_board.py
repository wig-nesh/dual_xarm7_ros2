#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2


def main() -> None:
    parser = argparse.ArgumentParser(description='generate a print-ready charuco board png')
    parser.add_argument('--output', default='charuco_board.png')
    parser.add_argument('--squares-x', type=int, default=5)
    parser.add_argument('--squares-y', type=int, default=5)
    parser.add_argument('--square-length-mm', type=float, default=25.0)
    parser.add_argument('--marker-length-mm', type=float, default=18.75)
    parser.add_argument('--dictionary', default='DICT_4X4_50')
    parser.add_argument('--margin-mm', type=float, default=20.0)
    parser.add_argument('--dpi', type=int, default=300)
    args = parser.parse_args()

    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, args.dictionary))
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x, args.squares_y,
        args.square_length_mm / 1000.0, args.marker_length_mm / 1000.0, dictionary)

    pixels_per_mm = args.dpi / 25.4
    board_w_px = round(args.squares_x * args.square_length_mm * pixels_per_mm)
    board_h_px = round(args.squares_y * args.square_length_mm * pixels_per_mm)
    margin_px = round(args.margin_mm * pixels_per_mm)
    width = board_w_px + 2 * margin_px
    height = board_h_px + 2 * margin_px
    image = board.generateImage((board_w_px, board_h_px), margin_px, 1)
    canvas = cv2.copyMakeBorder(image, margin_px, margin_px, margin_px, margin_px,
                                cv2.BORDER_CONSTANT, value=(255, 255, 255))
    output = Path(args.output)
    cv2.imwrite(str(output), canvas)
    print(f'wrote {output} ({width}x{height} px)')
    print(f'board: {args.squares_x}x{args.squares_y} squares '
          f'of {args.square_length_mm} mm, total '
          f'{args.squares_x * args.square_length_mm}x{args.squares_y * args.square_length_mm} mm')
    print(f'print at exactly {args.dpi} dpi (or 100 percent scale), then measure a square '
          'with calipers and set square_length_m in config/handeye_calibration.yaml '
          'to the measured value')


if __name__ == '__main__':
    main()
