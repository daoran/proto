#!/usr/bin/env python3
"""
Script to generate a grid of AprilTags, aka AprilGrid.
"""
import argparse
from PIL import Image

# Config
NUM_ROWS = 10
NUM_COLS = 10
TAG_FAMILY = "tag36h11"
# TAG_FAMILY = "tagStandard41h12"
TAG_IMG_DIR = "./deps/src/apriltag-imgs"
TAG_SPACING = 1  # Space between tags in pixels
SAVE_PATH = "./aprilgrid.png"


def make_aprilgrid(
    tag_rows,
    tag_cols,
    tag_img_dir,
    tag_family,
    tag_spacing,
    output_path,
):
    """ Make AprilGrid """
    # Settings

    # AprilTag defaults
    tag_size = 8  # Default tag size in pixels

    # Create AprilGrid
    img_w = tag_size * tag_cols + tag_spacing * (tag_cols - 1)
    img_h = tag_size * tag_rows + tag_spacing * (tag_rows - 1)
    img_scale = 10
    mosaic_size = (img_w, img_h)
    mosaic_bg_color = (255, 255, 255)
    mosaic = Image.new(mode='RGB', size=mosaic_size, color=mosaic_bg_color)

    tag_id = 0
    for i in range(tag_rows - 1, -1, -1):
        for j in range(tag_cols):
            img_prefix = tag_family.replace('Circle', '')
            img_prefix = tag_family.replace('Custom', '')
            img_prefix = tag_family.replace('Standard', '')
            img_prefix = img_prefix.replace('h', '_')

            tag_id_str = str(tag_id).zfill(5)
            tag_img_path = f'{tag_img_dir}/{tag_family}/{img_prefix}_{tag_id_str}.png'
            tag_img = Image.open(tag_img_path)

            tag_w, tag_h = tag_img.size
            tag_img = tag_img.crop((1, 1, tag_w - 1, tag_h - 1))

            px = (tag_size * j) + tag_spacing * j
            py = (tag_size * i) + tag_spacing * i
            mosaic.paste(tag_img, box=(px, py))
            tag_id += 1

    # Scale image and save
    mosaic_size_px = (img_w * img_scale, img_h * img_scale)
    mosaic = mosaic.resize(mosaic_size_px, Image.NEAREST)
    mosaic.save(output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag_rows", default=NUM_ROWS)
    parser.add_argument("--tag_cols", default=NUM_COLS)
    parser.add_argument("--tag_img_dir", default=TAG_IMG_DIR)
    parser.add_argument("--tag_family", default=TAG_FAMILY)
    parser.add_argument("--tag_spacing", default=TAG_SPACING)
    parser.add_argument("--output_path", default=SAVE_PATH)
    args = parser.parse_args()

    make_aprilgrid(
        tag_rows=args.tag_rows,
        tag_cols=args.tag_cols,
        tag_img_dir=args.tag_img_dir,
        tag_family=args.tag_family,
        tag_spacing=args.tag_spacing,
        output_path=args.output_path,
    )
