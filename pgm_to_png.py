#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from pathlib import Path
from PIL import Image


def pgm_to_png(input_path: str, output_path: str = None):
    input_file = Path(input_path)

    if not input_file.exists():
        raise FileNotFoundError(f"找不到输入文件: {input_file}")

    if output_path is None:
        output_file = input_file.with_suffix(".png")
    else:
        output_file = Path(output_path)

    # 读取 PGM 并保存为 PNG
    with Image.open(input_file) as img:
        img.save(output_file, format="PNG")

    print(f"转换完成: {input_file} -> {output_file}")


def main():
    if len(sys.argv) < 2:
        print("用法:")
        print("  python pgm_to_png.py input.pgm [output.png]")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else None

    pgm_to_png(input_path, output_path)


if __name__ == "__main__":
    main()

