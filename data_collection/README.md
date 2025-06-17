

# Data Collection Setup

This guide walks you through launching the full system, including CAN communication, robot controller, iPhone and other cameras, VR initialization, and data collection.

For VR setup, please first read <a href="../async_point_cloud_render/">Async Point Cloud Rendering</a>.
For gello software install, please read <a href="./gello_software/">gello_software</a>.



## Step-by-Step Instructions

```bash
cd data_collection


# Step 1: Bring up CAN interface
chmod +x bringup_can.sh
./bringup_can.sh

# Step 2: Start ROS core (in a new terminal)
chmod +x roscore.sh
./roscore.sh

# Step 3: Launch robot controller (in a new terminal)
chmod +x robot_controller.sh
./robot_controller.sh

# Step 4: Start iPhone camera stream (in a new terminal)
chmod +x iphone_camera.sh
./iphone_camera.sh

# Step 5: Start other external cameras (in a new terminal)
chmod +x other_cameras.sh
./other_cameras.sh

# Step 6: Initialize robot neck and VR (in a new terminal)
chmod +x vr.sh
./vr.sh
# Put on the VR headset, sit still,
# and press "~" when ready to start moving the neck


# Step 7: Start Gello for manual teleoperation (in a new terminal)
chmod +x gello.sh
./gello.sh

# Step 8: Begin data collection (in another terminal)
chmod +x data.sh
./data.sh
```


# Data Processing

The collected raw data is saved as ROS bag files.
Before model training, we provide a script for data preprocessing.

```bash
cd data_collection

mamba activate via

python raw_data_process.py --name directory-to-save-zarr-data --data directory-to-load-raw-data
