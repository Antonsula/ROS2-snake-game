#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import os

SCORE_FILE = os.path.expanduser("~/.ros2_snake_highscore.txt")
GRID_WIDTH = 20
GRID_HEIGHT = 20


class SnakeGameNode(Node):
    def __init__(self):
        super().__init__('snake_game_node')

        # Initial snake
        self.snake = [(10, 10), (9, 10), (8, 10)]
        self.direction = (1, 0)  # moving right
        self.pending_direction = self.direction
        self.num_obstacles = 5
        self.obstacles = self.spawn_obstacles()
        self.food = self.spawn_food()
        self.score = 0
        self.high_score = 0
        self.load_high_score()
        self.started = False
        self.game_over = False
        self.new_high_score = False
        self.base_speed = 0.2
        self.max_speed = 0.08
        self.speed_step = 0.01
        self.control_mode = None 
        self.timer_period = self.base_speed
        
        
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
        self.timer = self.create_timer(self.timer_period, self.update_game)

        self.get_logger().info("Snake game node started")

    def spawn_food(self):
        while True:
            pos = (
                random.randint(0, GRID_WIDTH - 1),
                random.randint(0, GRID_HEIGHT - 1)
            )

            if (
                pos not in self.snake and
                pos not in self.obstacles
            ):
                return pos

    def spawn_obstacles(self):
        obstacles = set()

        while len(obstacles) < self.num_obstacles:
            pos = (
                random.randint(0, GRID_WIDTH - 1),
                random.randint(0, GRID_HEIGHT - 1)
            )

            if pos not in self.snake:
                obstacles.add(pos)

        return list(obstacles)

    def direction_callback(self, msg):

        # Allow AI commands only in AI mode
        if msg.data in ['UP', 'DOWN', 'LEFT', 'RIGHT'] and self.control_mode == "AI":
            self.pending_direction = {
                'UP': (0, -1),
                'DOWN': (0, 1),
                'LEFT': (-1, 0),
                'RIGHT': (1, 0),
            }[msg.data]
            return
        # Start options
        if msg.data == "START_HUMAN":
            self.started = True
            self.control_mode = "HUMAN"
            self.get_logger().info("Game started in HUMAN mode")
            return

        if msg.data == "START_AI":
            self.started = True
            self.control_mode = "AI"
            self.get_logger().info("Game started in AI mode")
            return

        if msg.data == "RESET":
            self.reset_game()
            return

        # Ignore movement if not started
        if not self.started:
            return

        # Ignore human input if AI mode
        if self.control_mode == "AI":
            return

        # Normal WASD handling below
        mapping = {
            'UP': (0, -1),
            'DOWN': (0, 1),
            'LEFT': (-1, 0),
            'RIGHT': (1, 0),
        }

        if msg.data not in mapping:
            return

        new_dir = mapping[msg.data]
        if (new_dir[0] == -self.direction[0] and
            new_dir[1] == -self.direction[1]):
            return

        self.pending_direction = new_dir

    def increase_speed(self):
        new_period = max(
            self.max_speed,
            self.timer_period - self.speed_step
        )

        if new_period < self.timer_period:
            self.timer.cancel()
            self.timer_period = new_period
            self.timer = self.create_timer(self.timer_period, self.update_game)

            self.get_logger().info(
                f"Speed increased: {self.timer_period:.2f}s"
            )

    def publish_state(self, state):
        msg = String()
        msg.data = (
            f"{self.snake}|{self.food}|{self.obstacles}|"
            f"{self.score}|{self.high_score}|{state}|"
            f"{self.timer_period}|{self.control_mode}"
        )
        self.state_pub.publish(msg)

    def update_game(self):
        if not self.started:
            self.publish_state("START")
            return

        if self.game_over:
            state = "NEW_HIGH_SCORE" if self.new_high_score else "LOSE"
            self.publish_state(state)
            return
        self.direction = self.pending_direction
        head_x, head_y = self.snake[0]
        dx, dy = self.direction
        new_head = (head_x + dx, head_y + dy)

        if (
            new_head in self.snake or
            new_head in self.obstacles or
            new_head[0] < 0 or new_head[0] >= GRID_WIDTH or
            new_head[1] < 0 or new_head[1] >= GRID_HEIGHT
        ):
            self.get_logger().info("YOU LOST")
            if self.new_high_score:
                self.save_high_score()
            self.game_over = True
            return

        self.snake.insert(0, new_head)

        if new_head == self.food:
            self.get_logger().info(f"Score: {self.score}")
            self.score += 10
            if self.score > self.high_score:
                self.high_score = self.score
                self.new_high_score = True
            self.food = self.spawn_food()
            self.increase_speed()
        else:
            self.snake.pop()

        self.publish_state("PLAYING")

    def reset_game(self):
        self.snake = [(10, 10), (9, 10), (8, 10)]
        self.direction = (1, 0)
        self.pending_direction = self.direction
        self.obstacles = self.spawn_obstacles()
        self.food = self.spawn_food()
        self.score = 0
        self.timer.cancel()
        self.timer_period = self.base_speed
        self.timer = self.create_timer(self.timer_period, self.update_game)
        self.game_over = False
        self.new_high_score = False
        self.started = False
        
        self.get_logger().info("Game restarted")

    def load_high_score(self):
        try:
            with open(SCORE_FILE, "r") as f:
                self.high_score = int(f.read().strip())
                self.get_logger().info(
                    f"Loaded high score: {self.high_score}"
                )
        except Exception:
            self.high_score = 0


    def save_high_score(self):
        try:
            with open(SCORE_FILE, "w") as f:
                f.write(str(self.high_score))
        except Exception as e:
            self.get_logger().warn(
                f"Could not save high score: {e}"
            )

def main():
    rclpy.init()
    node = SnakeGameNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
