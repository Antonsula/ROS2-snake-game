# ROS 2 Snake Game

A simple graphical Snake game implemented in **ROS 2 (rclpy)** and **Pygame**, designed to run on **WSL with XLaunch**.

## Features
- Modular ROS 2 nodes
- Graphical rendering using Pygame
- Keyboard control (W A S D)
- Score tracking
- Clean separation of logic and rendering

## Architecture
- `snake_game_node` — game logic (movement, collisions, score)
- `renderer_node` — graphics + keyboard input
- `snake_controller_node` — AI controller (used in AI mode)

## Requirements
- ROS 2 (Humble or later)
- Python 3
- pygame
- WSL + X11 (XLaunch)

## Build & Run (run renderer first and then snake_game in different terminals)
```bash
colcon build
source install/setup.bash
ros2 run ros2_snake snake_game
ros2 run ros2_snake renderer
```

## Run With Docker
1. Build image:
```bash
docker compose build
```

2. If you are on Linux host, allow Docker to use your X server:
```bash
xhost +local:docker
```

3. Start the game:
```bash
docker compose up
```

This starts `snake_game`, `renderer`, and `controller` inside the container.

### WSL + XLaunch note
- Start XLaunch with access control disabled.
- Set `DISPLAY` before running compose:
```bash
export DISPLAY=host.docker.internal:0.0
docker compose up
```
