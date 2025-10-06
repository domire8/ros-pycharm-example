FROM ghcr.io/aica-technology/ros2-ws:humble

RUN sudo apt-get update && sudo apt-get install -y ros-humble-turtlesim && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER}/ros2_ws
COPY --chown=${USER} ./turtle_example ./src/turtle_example
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
