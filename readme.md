# ROS 2 Snake Game 🐍🎮

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

## Requirements
- ROS 2 (Humble or later)
- Python 3
- pygame
- WSL + X11 (XLaunch)

## Build & Run
```bash
colcon build
source install/setup.bash
ros2 run ros2_snake snake_game
ros2 run ros2_snake renderer
run renderer first and then snake_game
