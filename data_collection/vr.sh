#!/bin/bash

# Load mamba into the shell
eval "$(mamba shell hook --shell bash)"

# Activate the via environment
mamba activate via

# Set ROS environment variables
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1

cd ../src

python init_neck.py

sleep 3
# Run the Python script
python VisionPro_node.py

