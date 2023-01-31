#!/bin/bash
set -e

EUROC_DATA_PATH=/data/euroc
EUROC_URL=http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset

EUROC_CAM_CALIB=calibration_datasets/cam_april/cam_april.zip
EUROC_IMU_CALIB=calibration_datasets/imu_april/imu_april.zip
EUROC_MH_01=machine_hall/MH_01_easy/MH_01_easy.zip
EUROC_MH_02=machine_hall/MH_02_easy/MH_02_easy.zip
EUROC_MH_03=machine_hall/MH_03_medium/MH_03_medium.zip
EUROC_MH_04=machine_hall/MH_04_difficult/MH_04_difficult.zip
EUROC_MH_05=machine_hall/MH_05_difficult/MH_05_difficult.zip
EUROC_V1_01=vicon_room1/V1_01_easy/V1_01_easy.zip
EUROC_V1_02=vicon_room1/V1_02_medium/V1_02_medium.zip
EUROC_V1_03=vicon_room1/V1_03_difficult/V1_03_difficult.zip
EUROC_V2_01=vicon_room2/V2_01_easy/V2_01_easy.zip
EUROC_V2_02=vicon_room2/V2_02_medium/V2_02_medium.zip
EUROC_V2_03=vicon_room2/V2_03_difficult/V2_03_difficult.zip

EUROC_DATASETS=(
  "${EUROC_CAM_CALIB}"
  "${EUROC_IMU_CALIB}"
	"${EUROC_MH_01}"
	"${EUROC_MH_02}"
	"${EUROC_MH_03}"
	"${EUROC_MH_04}"
	"${EUROC_MH_05}"
	"${EUROC_V1_01}"
	"${EUROC_V1_02}"
	"${EUROC_V1_03}"
	"${EUROC_V2_01}"
	"${EUROC_V2_02}"
	"${EUROC_V2_03}"
)

function download_dataset() {
  DATASET_DIR="$(basename $1)";
  DATASET_DIR="${DATASET_DIR/.zip/}";
  DATASET_DIR="${DATASET_DIR/_easy/}";
  DATASET_DIR="${DATASET_DIR/_medium/}";
  DATASET_DIR="${DATASET_DIR/_difficult/}";
  DATASET_DIR="${EUROC_DATA_PATH}/$DATASET_DIR";

  if [ ! -d "${DATASET_DIR}" ]; then
    echo "Downloading [$DATASET_DIR]";
    mkdir -p "$DATASET_DIR";
    wget -nc "${EUROC_URL}/$1" -O "$DATASET_DIR.zip"

    echo "Unzipping [$DATASET_DIR.zip]";
    unzip -oq "$DATASET_DIR.zip" -d "$DATASET_DIR"
    rm "$DATASET_DIR.zip"
  fi
}

for DATASET in "${EUROC_DATASETS[@]}"; do
  download_dataset "${DATASET}"
done
