#!/bin/sh

bags=("$1")
topic=("$2")

cd ../Livox-SDK/ws_livox/data/lidar/
# 检查 .bag 文件是否存在
if [[ -f "$1" ]]; then
  echo "Unpackage ROSBAG: $1"
  rosrun pcl_ros bag_to_pcd $1 $2 ../pcdFiles
else
  echo "Error: $1 does not exist. Skipping..."
fi
