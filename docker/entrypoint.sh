#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/ros2_snake_ws/install/setup.bash

exec "$@"
