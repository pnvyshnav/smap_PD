#!/bin/bash

# download EuRoC Vicon Room 1 01 Dataset
echo "Downloading EuRoC MAV Dataset..."
#wget -nc http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.bag -O dataset/V1_01_easy/V1_01_easy.bag
wget -nc http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip -O dataset/V1_01_easy/V1_01_easy.zip
unzip -j "dataset/V1_01_easy/V1_01_easy.zip" "mav0/pointcloud0/data.ply" -d "dataset/V1_01_easy"
rm dataset/V1_01_easy/V1_01_easy.zip