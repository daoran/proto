#!/usr/bin/env python3
"""
Script to generate a grid of AprilTags, aka AprilGrid.
"""
from PIL import Image

# Config
NUM_ROWS = 5
NUM_COLS = 5
TAG_FAMILY = "tagStandard41h12"
TAG_IMG_DIR = "./apriltag-imgs"
TAG_SPACING = 0  # Space between tags in pixels
SAVE_PATH = "./aprilgrid.png"


def make_aprilgrid(num_rows, num_cols, tag_img_dir, tag_family, tag_spacing,
                   save_path):
  """ Make AprilGrid """
  # AprilTag defaults
  tag_size = 9  # Default tag size in pixels
  tag_width = 5  # Tag corner detection with in pixels

  # Create AprilGrid
  img_w = tag_size * num_cols + tag_spacing * (num_cols - 1)
  img_h = tag_size * num_rows + tag_spacing * (num_rows - 1)
  img_scale = 10
  mosaic = Image.new(mode='RGB', size=(img_w, img_h))

  tag_id = 0
  for i in range(num_rows):
    for j in range(num_cols):
      img_prefix = tag_family.replace('Circle', '')
      img_prefix = tag_family.replace('Custom', '')
      img_prefix = tag_family.replace('Standard', '')
      img_prefix = img_prefix.replace('h', '_')

      tag_id_str = str(tag_id).zfill(5)
      tag_img_path = f'{tag_img_dir}/{tag_family}/{img_prefix}_{tag_id_str}.png'
      tag_img = Image.open(tag_img_path)

      px = (tag_size * j) + tag_spacing * j
      py = (tag_size * i) + tag_spacing * i
      mosaic.paste(tag_img, box=(px, py))
      tag_id += 1

  # Scale image and save
  mosaic_size_px = (img_w * img_scale, img_h * img_scale)
  mosaic = mosaic.resize(mosaic_size_px, Image.NEAREST)
  mosaic.save(save_path)


if __name__ == "__main__":
  make_aprilgrid(NUM_ROWS, NUM_COLS, TAG_IMG_DIR, TAG_FAMILY, TAG_SPACING,
                 SAVE_PATH)
