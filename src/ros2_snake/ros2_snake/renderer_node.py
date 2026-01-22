#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import ast
import random
CELL_SIZE = 25
GRID_SIZE = 20
WINDOW_SIZE = CELL_SIZE * GRID_SIZE


class RendererNode(Node):
    def __init__(self):
        super().__init__('renderer_node')
        self.confetti = []
        for _ in range(200):
            self.confetti.append([
                random.randint(0, WINDOW_SIZE),
                random.randint(-WINDOW_SIZE, 0),
                random.randint(2, 6),
                (random.randint(100,255), random.randint(100,255), random.randint(100,255))
            ])

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
    def draw_win_text(self, score):
        font = pygame.font.SysFont(None, 60)
        text = font.render("CONGRATULATIONS!", True, (255, 0, 0))
        rect = text.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2))
        self.screen.blit(text, rect)

        score_font = pygame.font.SysFont(None, 40)
        score_text = score_font.render(f"Final Score: {score}", True, (255, 255, 255))
        score_rect = score_text.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2 + 60))
        self.screen.blit(score_text, score_rect)
        hint_font = pygame.font.SysFont(None, 28)
        hint = hint_font.render("Press R to restart", True, (255, 255, 255))
        hint_rect = hint.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2 + 110))
        self.screen.blit(hint, hint_rect)


    def draw_confetti(self):
        for conf in self.confetti:
            conf[1] += conf[2]
            if conf[1] > WINDOW_SIZE:
                conf[1] = random.randint(-50, 0)
                conf[0] = random.randint(0, WINDOW_SIZE)

            pygame.draw.circle(
                self.screen,
                conf[3],
                (conf[0], conf[1]),
                4
            )

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
                elif event.key == pygame.K_r:
                     msg.data = 'RESET'
                else:
                    return

                self.direction_pub.publish(msg)

    def draw(self, msg):
        parts = msg.data.split('|')

        # Safety: wait for valid messages
        if len(parts) < 4:
            return

        snake_str, food_str, score_str, state = parts
        snake = ast.literal_eval(snake_str)
        food = ast.literal_eval(food_str)
        score = int(score_str)

        self.screen.fill((30, 30, 30))

        # WIN screen
        if state == "WIN":
            self.draw_confetti()
            self.draw_win_text(score)
            pygame.display.flip()
            return

        # LOSE screen
        if state == "LOSE":
            self.draw_lose_text(score)
            pygame.display.flip()
            return

        # PLAYING state
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

    def draw_lose_text(self, score):
        font = pygame.font.SysFont(None, 72)
        text = font.render("GAME OVER", True, (255, 0, 0))
        rect = text.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2))
        self.screen.blit(text, rect)

        score_font = pygame.font.SysFont(None, 40)
        score_text = score_font.render(f"Score: {score}", True, (255, 255, 255))
        score_rect = score_text.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2 + 60))
        self.screen.blit(score_text, score_rect)

        hint_font = pygame.font.SysFont(None, 28)
        hint = hint_font.render("Press R to restart", True, (255, 255, 255))
        hint_rect = hint.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2 + 110))
        self.screen.blit(hint, hint_rect)


def main():
    rclpy.init()
    node = RendererNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
