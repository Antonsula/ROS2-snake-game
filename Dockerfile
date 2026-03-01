FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspaces/ros2_snake_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pygame \
    && rm -rf /var/lib/apt/lists/*

COPY src ./src
COPY docker ./docker
COPY docker/entrypoint.sh /entrypoint.sh

RUN source /opt/ros/humble/setup.bash \
    && colcon build

RUN chmod +x /entrypoint.sh /workspaces/ros2_snake_ws/docker/run_snake.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/workspaces/ros2_snake_ws/docker/run_snake.sh"]
