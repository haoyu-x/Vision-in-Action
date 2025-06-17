#!/bin/bash
export ROS_MASTER_URI=http://localhost:11311/ export ROS_HOSTNAME=127.0.0.1 export ROS_IP=127.0.0.1




eval "$(conda shell.bash hook)"
# Activate the conda environment
conda activate ros_env

# Navigate to the gello directory
cd ~/vision-in-action/src

# Run the Python script
python read_exo_iphone_camera.py

