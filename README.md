This project is part of a larger system designed to detect driver drowsiness using Large Language Models (LLMs). The Data Capture Model is responsible for collecting real-time driving behavior and positional data from the CARLA simulator and exporting it in JSON format for downstream processing by a neural network.

- Project Overview
The data capture module performs the following tasks:

1. Connects to the CARLA simulator
   
2. Identifies and tracks the ego vehicle
   
3. Extracts key data, including:
- Steering angle
- Lane center position
- Vehicle's lateral deviation from the lane center

4. Logs the extracted information at 20 Hz

5. Saves data incrementally to a .json file for further analysis

This module forms the data acquisition layer of a broader drowsiness detection pipeline, enabling LLM-based systems to reason over driver state using sensor-derived features.


- Requirements
1. ROS 2 Humble
2. CARLA Simulator 0.9.14
3. Python 3.8+
4. rclpy, carla, math, json, os

#Getting started:

1. Ensure that the CARLA simulator is running before launching the ROS 2 node.

2. Running the Data Publisher Node
# Navigate to your ROS 2 workspace
cd ~/ros_data_capture_ws

# Build the workspace
colcon build

# Source the setup script
source install/setup.bash

# Run the data_publisher node
ros2 run data_capture data_publisher

3. JSON logs will be saved every 10 frames to:

~/Documents/carla_data_log.json



Authors
Usama Mehdi & Team
Project for LLM Driver Drowsiness Detection System
Technische Hochschule Ingolstadt




