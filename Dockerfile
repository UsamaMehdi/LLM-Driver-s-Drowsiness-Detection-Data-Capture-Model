FROM osrf/ros:humble-desktop

# Customizable environment variables
ENV PACKAGE_NAME=data_capture
ENV NODE_NAME=data_capture_node

# ROS2 specific variables
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN sudo apt-get update && sudo apt-get upgrade -y
RUN sudo apt-get install -y \
    wget \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf-transformations && \
    rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/gezp/carla_ros/releases/download/carla-0.9.14-ubuntu-22.04/carla-0.9.14-cp310-cp310-linux_x86_64.whl
RUN python3 -m pip install --no-cache-dir carla-0.9.14-cp310-cp310-linux_x86_64.whl

COPY /src /opt/ros2_ws/src
WORKDIR /opt/ros2_ws
RUN rosdep install -i --from-path src --rosdistro humble -y
RUN colcon build --cmake-force-configure --packages-select $PACKAGE_NAME

CMD ["bash", "-c", "source install/setup.bash && ros2 run $PACKAGE_NAME $NODE_NAME"]