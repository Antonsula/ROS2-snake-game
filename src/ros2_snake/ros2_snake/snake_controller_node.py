#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
import random
import math

GRID_WIDTH = 20
GRID_HEIGHT = 20


class SnakeController(Node):
    def __init__(self):
        super().__init__('snake_controller')

        self.state_sub = self.create_subscription(
            String,
            '/game_state',
            self.on_state,
            10
        )

        self.cmd_pub = self.create_publisher(
            String,
            '/direction',
            10
        )

        self.current_direction = None

        self.moves = {
            'UP': (0, -1),
            'DOWN': (0, 1),
            'LEFT': (-1, 0),
            'RIGHT': (1, 0),
        }

        self.opposite = {
            'UP': 'DOWN',
            'DOWN': 'UP',
            'LEFT': 'RIGHT',
            'RIGHT': 'LEFT',
        }

        self.get_logger().info("Snake AI controller started")

    def on_state(self, msg):
        parts = msg.data.split('|')
        if len(parts) < 8:
            return

        snake = ast.literal_eval(parts[0])
        food = ast.literal_eval(parts[1])
        obstacles = ast.literal_eval(parts[2])
        state = parts[5]
        control_mode = parts[7]

        # 🚫 Do nothing unless AI mode is active
        if state != "PLAYING" or control_mode != "AI":
            return

        direction = self.decide_move(snake, food, obstacles)
        if direction is None:
            return

        self.current_direction = direction
        cmd = String()
        cmd.data = direction
        self.cmd_pub.publish(cmd)


    def decide_move(self, snake, food, obstacles):
        head_x, head_y = snake[0]
        fx, fy = food

        safe_moves = []

        for move, (dx, dy) in self.moves.items():
            # Prevent reversing
            if self.current_direction and move == self.opposite[self.current_direction]:
                continue

            nx, ny = head_x + dx, head_y + dy

            # Wall collision
            if nx < 0 or nx >= GRID_WIDTH or ny < 0 or ny >= GRID_HEIGHT:
                continue

            # Obstacle collision
            if (nx, ny) in obstacles:
                continue

            # Self collision (ignore last tail cell since it moves)
            if (nx, ny) in snake[:-1]:
                continue

            safe_moves.append(move)

        if not safe_moves:
            return None  # no legal move

        # Prefer moves that reduce distance to food
        def distance(move):
            dx, dy = self.moves[move]
            nx, ny = head_x + dx, head_y + dy
            return abs(nx - fx) + abs(ny - fy)

        safe_moves.sort(key=distance)

        return safe_moves[0]


def main():
    rclpy.init()
    node = SnakeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
