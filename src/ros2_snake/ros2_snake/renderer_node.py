#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import ast
import random
import os
from ament_index_python.packages import get_package_share_directory

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
        pygame.display.set_caption("Antonsula's snake-game")


        pkg_share = get_package_share_directory('ros2_snake')
        image_path = os.path.join(pkg_share, 'assets', 'background.jpg')

        self.start_bg = pygame.image.load(image_path).convert()
        self.start_bg = pygame.transform.scale(
            self.start_bg,
            (WINDOW_SIZE, WINDOW_SIZE)
        )

        self.direction_pub = self.create_publisher(
            String,
            '/direction',
            10
        )

        self.state_sub = self.create_subscription(
            String,
            '/game_state',
            self.on_state,
            10
        )

        self.last_state_time = pygame.time.get_ticks()
        self.move_duration = 150 
        self.prev_snake = None
        self.latest_state = None
        self.target_snake = None

        self.latest_time = pygame.time.get_ticks()
        self.timer = self.create_timer(0.016, self.render)

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

    def draw_difficulty_screen(self):
        self.screen.blit(self.start_bg, (0, 0))

        overlay = pygame.Surface((WINDOW_SIZE, WINDOW_SIZE))
        overlay.set_alpha(160)
        overlay.fill((0, 0, 0))
        self.screen.blit(overlay, (0, 0))

        font = pygame.font.SysFont(None, 36)
        title = font.render("Choose Difficulty", True, (255, 215, 0))
        self.screen.blit(title, (WINDOW_SIZE//2 - 120, 120))

        options = [
            "E - Easy 5 (obstacles, low speed)",
            "M - Medium (10 obstacles, medium speed)",
            "H - Hard (20 obstacles, high speed)",
        ]

        for i, text in enumerate(options):
            line = font.render(text, True, (255, 255, 255))
            self.screen.blit(line, (WINDOW_SIZE//2 - 160, 180 + i*40))

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
    def on_state(self, msg):
        self.latest_state = msg.data


    def render(self):
        self.handle_events()
        if self.latest_state is None:
            return
        self.draw(self.latest_state)
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
                elif event.key == pygame.K_1:
                    msg.data = 'START_HUMAN'
                elif event.key == pygame.K_2:
                    msg.data = 'START_AI'
                elif event.key == pygame.K_e:
                    msg.data = "DIFF_EASY"
                elif event.key == pygame.K_m:
                    msg.data = "DIFF_MEDIUM"
                elif event.key == pygame.K_h:
                    msg.data = "DIFF_HARD"
                else:
                    return

                self.direction_pub.publish(msg)
    def draw_eyes(self, cx, cy, dx, dy):
        eye_offset = CELL_SIZE // 6
        eye_radius = 3

        # Perpendicular direction for eye spacing
        px, py = -dy, dx

        eye1 = (
            cx + dx * 6 + px * eye_offset,
            cy + dy * 6 + py * eye_offset
        )
        eye2 = (
            cx + dx * 6 - px * eye_offset,
            cy + dy * 6 - py * eye_offset
        )

        pygame.draw.circle(self.screen, (255, 255, 255), eye1, eye_radius)
        pygame.draw.circle(self.screen, (255, 255, 255), eye2, eye_radius)

        pygame.draw.circle(self.screen, (0, 0, 0), eye1, 1)
        pygame.draw.circle(self.screen, (0, 0, 0), eye2, 1)
    
    def draw(self, state_string):

        parts = state_string.split('|')

        if len(parts) == 9:
            snake_str, food_str, obstacles_str, score_str, high_score_str, state, period_str, mode, difficulty = parts
            self.move_duration = float(period_str) * 1000
        elif len(parts) == 8:
            snake_str, food_str, obstacles_str, score_str, high_score_str, state, period_str, mode = parts
            self.move_duration = float(period_str) * 1000
            difficulty = None
        elif len(parts) == 7:
            snake_str, food_str, obstacles_str, score_str, high_score_str, state, period_str = parts
            mode = None
            difficulty = None
        elif len(parts) == 6:
            snake_str, food_str, score_str, high_score_str, state, period_str = parts
            self.move_duration = float(period_str) * 1000
            obstacles_str = "[]"
        elif len(parts) == 5:
            snake_str, food_str, score_str, high_score_str, state = parts
            obstacles_str = "[]"
        elif len(parts) == 4:
            snake_str, food_str, score_str, state = parts
            high_score_str = score_str
            obstacles_str = "[]"
        else:
            self.get_logger().warn(f"Bad state message: {state_string}")
            return


        # Parse state FIRST
        snake = ast.literal_eval(snake_str)
        food = ast.literal_eval(food_str)
        
        score = int(score_str)
        high_score = int(high_score_str)

        current_time = pygame.time.get_ticks()

        # First-ever frame
        if self.target_snake is None:
            self.prev_snake = snake
            self.target_snake = snake
            self.last_state_time = current_time

        # New logical step arrived from game node
        elif snake != self.target_snake:
            self.prev_snake = self.target_snake
            self.target_snake = snake
            self.last_state_time = current_time


        # Interpolation factor
        elapsed = current_time - self.last_state_time
        t = min(elapsed / self.move_duration, 1.0)

        if state == "CHOOSE_DIFFICULTY":
            self.draw_difficulty_screen()
            pygame.display.flip()
            return

        # Handle non-playing screens early
        if state == 'START':
            self.draw_start_screen(high_score)
            pygame.display.flip()
            return

        if state == 'NEW_HIGH_SCORE':
            self.screen.fill((30, 30, 30))
            self.draw_confetti()
            self.draw_new_high_score_text(score)
            pygame.display.flip()
            return

        if state == 'LOSE':
            self.screen.fill((30, 30, 30))
            self.draw_lose_text(score)
            pygame.display.flip()
            return
        
        obstacles = ast.literal_eval(obstacles_str)
        # Determine head direction (for eyes)
        head_x, head_y = snake[0]
        if len(snake) > 1:
            neck_x, neck_y = snake[1]
            dir_x = head_x - neck_x
            dir_y = head_y - neck_y
        else:
            dir_x, dir_y = 1, 0

        self.screen.fill((30, 30, 30))

        for ox, oy in obstacles:
            x = ox * CELL_SIZE
            y = oy * CELL_SIZE

            # Yellow base
            pygame.draw.rect(
                self.screen,
                (255, 215, 0),  # yellow
                (x, y, CELL_SIZE, CELL_SIZE),
                border_radius=4
            )

            # Black X (cross)
            thickness = 3
            padding = 5

            pygame.draw.line(
                self.screen,
                (0, 0, 0),
                (x + padding, y + padding),
                (x + CELL_SIZE - padding, y + CELL_SIZE - padding),
                thickness
            )

            pygame.draw.line(
                self.screen,
                (0, 0, 0),
                (x + CELL_SIZE - padding, y + padding),
                (x + padding, y + CELL_SIZE - padding),
                thickness
            )

        
        
        # SMOOTH INTERPOLATED SNAKE DRAW
        for i, (x, y) in enumerate(self.target_snake):

            if i < len(self.prev_snake):
                px, py = self.prev_snake[i]
            else:
                px, py = x, y

            ix = px + (x - px) * t
            iy = py + (y - py) * t

            draw_x = int(ix * CELL_SIZE)
            draw_y = int(iy * CELL_SIZE)

            pygame.draw.rect(
                self.screen,
                (0, 180, 0),
                (draw_x, draw_y, CELL_SIZE, CELL_SIZE),
                border_radius=6
            )

            if i == 0:
                cx = draw_x + CELL_SIZE // 2
                cy = draw_y + CELL_SIZE // 2
                self.draw_eyes(cx, cy, dir_x, dir_y)

        # Food (no interpolation needed)
        fx, fy = food
        pygame.draw.rect(
            self.screen,
            (255, 0, 0),
            (fx * CELL_SIZE, fy * CELL_SIZE, CELL_SIZE, CELL_SIZE),
            border_radius=20
        )

        font = pygame.font.SysFont(None, 32)
        self.screen.blit(
            font.render(f"Score: {score}", True, (255, 255, 255)),
            (5, 5)
        )
        self.screen.blit(
            font.render(f"High Score: {high_score}", True, (255, 215, 0)),
            (5, 35)
        )

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

    def draw_new_high_score_text(self, score):
        font = pygame.font.SysFont(None, 60)
        text = font.render("NEW HIGH SCORE!", True, (255, 215, 0))
        rect = text.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2))
        self.screen.blit(text, rect)

        score_font = pygame.font.SysFont(None, 40)
        score_text = score_font.render(f"Score: {score}", True, (255, 255, 255))
        score_rect = score_text.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2 + 60))
        self.screen.blit(score_text, score_rect)

        hint_font = pygame.font.SysFont(None, 28)
        hint = hint_font.render("Press R to try again", True, (255, 255, 255))
        hint_rect = hint.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2 + 110))
        self.screen.blit(hint, hint_rect)

    def draw_start_screen(self, high_score):
        self.screen.blit(self.start_bg, (0, 0))
        overlay = pygame.Surface((WINDOW_SIZE, WINDOW_SIZE))
        overlay.set_alpha(120)
        overlay.fill((0, 0, 0))
        self.screen.blit(overlay, (0, 0))
        title_font_1 = pygame.font.SysFont(None, 50)
        line1 = title_font_1.render("Antonsula's snake game", True, (0, 200, 0))

        line1_rect = line1.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2 - 80))

        self.screen.blit(line1, line1_rect)

        info_font = pygame.font.SysFont(None, 36)

        hs_font = pygame.font.SysFont(None, 28)
        hs = hs_font.render(f"High Score: {high_score}", True, (255, 215, 0))
        hs_rect = hs.get_rect(center=(WINDOW_SIZE // 2, WINDOW_SIZE // 2 + 120))
        self.screen.blit(hs, hs_rect)
        info1 = info_font.render("1 – Play yourself (WASD)", True, (255, 255, 255))
        info2 = info_font.render("2 – Watch AI play", True, (255, 255, 255))

        self.screen.blit(info1, (WINDOW_SIZE//2 - 150, WINDOW_SIZE//2 + 20))
        self.screen.blit(info2, (WINDOW_SIZE//2 - 150, WINDOW_SIZE//2 + 60))

def main():
    rclpy.init()
    node = RendererNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
