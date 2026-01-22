#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

GRID_WIDTH = 20
GRID_HEIGHT = 20


class SnakeGameNode(Node):
    def __init__(self):
        super().__init__('snake_game_node')

        # Initial snake
        self.snake = [(10, 10), (9, 10), (8, 10)]
        self.direction = (1, 0)  # moving right
        self.food = self.spawn_food()
        self.score = 0
        # ROS interfaces
        self.dir_sub = self.create_subscription(
            String,
            '/direction',
            self.direction_callback,
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/game_state',
            10
        )

        # Game loop timer
        self.timer = self.create_timer(0.2, self.update_game)

        self.get_logger().info("Snake game node started")

    def spawn_food(self):
        return (
            random.randint(0, GRID_WIDTH - 1),
            random.randint(0, GRID_HEIGHT - 1)
        )

    def direction_callback(self, msg):
        mapping = {
            'UP': (0, -1),
            'DOWN': (0, 1),
            'LEFT': (-1, 0),
            'RIGHT': (1, 0),
        }

        if msg.data not in mapping:
            return

        new_dir = mapping[msg.data]
        current_dir = self.direction

        # Prevent reversing direction
        if (new_dir[0] == -current_dir[0] and
            new_dir[1] == -current_dir[1]):
            return  # illegal move, ignore

        self.direction = new_dir


    def update_game(self):
        head_x, head_y = self.snake[0]
        dx, dy = self.direction
        new_head = (head_x + dx, head_y + dy)

        # Wall or self collision
        if (
            new_head in self.snake or
            new_head[0] < 0 or new_head[0] >= GRID_WIDTH or
            new_head[1] < 0 or new_head[1] >= GRID_HEIGHT
        ):
            self.get_logger().info("GAME OVER")
            rclpy.shutdown()
            return

        self.snake.insert(0, new_head)

        if new_head == self.food:
            self.get_logger().info(f"Score: {self.score}")
            self.score += 10
            self.food = self.spawn_food()

        else:
            self.snake.pop()

        msg = String()
        msg.data = f"{self.snake}|{self.food}|{self.score}"
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = SnakeGameNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
