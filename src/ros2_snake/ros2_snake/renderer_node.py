#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import ast

CELL_SIZE = 25
GRID_SIZE = 20
WINDOW_SIZE = CELL_SIZE * GRID_SIZE


class RendererNode(Node):
    def __init__(self):
        super().__init__('renderer_node')

        pygame.init()
        self.screen = pygame.display.set_mode(
            (WINDOW_SIZE, WINDOW_SIZE)
        )
        pygame.display.set_caption("ROS2 Snake")

        self.direction_pub = self.create_publisher(
            String,
            '/direction',
            10
        )

        self.state_sub = self.create_subscription(
            String,
            '/game_state',
            self.draw,
            10
        )

        self.timer = self.create_timer(0.05, self.handle_events)

        self.get_logger().info("Renderer node started")

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return

            if event.type == pygame.KEYDOWN:
                msg = String()

                if event.key == pygame.K_w:
                    msg.data = 'UP'
                elif event.key == pygame.K_s:
                    msg.data = 'DOWN'
                elif event.key == pygame.K_a:
                    msg.data = 'LEFT'
                elif event.key == pygame.K_d:
                    msg.data = 'RIGHT'
                else:
                    return

                self.direction_pub.publish(msg)

    def draw(self, msg):
        snake_str, food_str, score_str = msg.data.split('|')
        snake = ast.literal_eval(snake_str)
        food = ast.literal_eval(food_str)
        score = int(score_str)
        self.screen.fill((30, 30, 30))

        for x, y in snake:
            pygame.draw.rect(
                self.screen,
                (0, 255, 0),
                (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            )

        fx, fy = food
        pygame.draw.rect(
            self.screen,
            (255, 0, 0),
            (fx * CELL_SIZE, fy * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        )
        font = pygame.font.SysFont(None, 32)
        score_surface = font.render(f"Score: {score}", True, (255, 255, 255))
        self.screen.blit(score_surface, (5, 5))
        pygame.display.flip()


def main():
    rclpy.init()
    node = RendererNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
