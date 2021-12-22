#!/bin/bash
set -e

DATA_DIR="/data/euroc/raw"
BASE_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/"

RAW_DATASETS=(
  "vicon_room1/V1_01_easy/V1_01_easy.zip"
)

# RAW_DATASETS=(
#   "machine_hall/MH_01_easy/MH_01_easy.zip"
#   "machine_hall/MH_02_easy/MH_02_easy.zip"
#   "machine_hall/MH_03_medium/MH_03_medium.zip"
#   "machine_hall/MH_04_difficult/MH_04_difficult.zip"
#   "machine_hall/MH_05_difficult/MH_05_difficult.zip"
#   "vicon_room1/V1_01_easy/V1_01_easy.zip"
#   "vicon_room1/V1_02_medium/V1_02_medium.zip"
#   "vicon_room1/V1_03_difficult/V1_03_difficult.zip"
#   "vicon_room2/V2_01_easy/V2_01_easy.zip"
#   "vicon_room2/V2_02_medium/V2_02_medium.zip"
#   "vicon_room2/V2_03_difficult/V2_03_difficult.zip"
# )

download_dataset() {
  SEQ_URI=$1

  ZIP_FILE=$(basename "$SEQ_URI")
  DS_FOLDER="${ZIP_FILE/.zip/}"
  DS_FOLDER="${DS_FOLDER/_easy/}"
  DS_FOLDER="${DS_FOLDER/_medium/}"
  DS_FOLDER="${DS_FOLDER/_hard/}"

  # Create data directory
  mkdir -p "$DATA_DIR"
  cd "$DATA_DIR"

  # Download
  if [ -d "$DS_FOLDER" ]; then
    return 0
  fi
  if [ ! -f "$(basename "$SEQ_URI")" ]; then
    wget "$BASE_URL/$SEQ_URI"
  fi

  # Unzip
  mkdir -p "$DS_FOLDER"
  cd "$DS_FOLDER"
  unzip -o ../"$(basename "$SEQ_URI")"
  rm -rf __MACOSX
  rm "../$(basename "$SEQ_URI")"

  return 0
}

for SEQ_URI in "${RAW_DATASETS[@]}"; do
  download_dataset "$SEQ_URI"
done
