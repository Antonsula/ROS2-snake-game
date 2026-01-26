#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
from collections import deque
GRID_WIDTH = 20
GRID_HEIGHT = 20


def build_hamiltonian_cycle(width, height):
    cycle = []
    for y in range(height):
        if y % 2 == 0:
            for x in range(width):
                cycle.append((x, y))
        else:
            for x in reversed(range(width)):
                cycle.append((x, y))
    return cycle


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

        # Hamiltonian setup
        self.cycle = build_hamiltonian_cycle(GRID_WIDTH, GRID_HEIGHT)
        self.index = {pos: i for i, pos in enumerate(self.cycle)}
        self.cycle_len = len(self.cycle)

        self.current_direction = None

        self.moves = {
            'UP': (0, -1),
            'DOWN': (0, 1),
            'LEFT': (-1, 0),
            'RIGHT': (1, 0),
        }

        self.get_logger().info("Hamiltonian Snake AI started")

    def on_state(self, msg):
        parts = msg.data.split('|')
        if len(parts) < 8:
            return

        snake = ast.literal_eval(parts[0])
        food = ast.literal_eval(parts[1])
        obstacles = ast.literal_eval(parts[2])
        state = parts[5]
        control_mode = parts[7]

        if state != "PLAYING" or control_mode != "AI":
            return

        move = self.decide_move(snake, food, obstacles)
        if move is None:
            return

        self.current_direction = move
        cmd = String()
        cmd.data = move
        self.cmd_pub.publish(cmd)

    # ---------------- LOGIC ---------------- #
    

    def can_reach_tail(self, snake, obstacles):
        """
        Check if head can reach tail without crossing body or obstacles.
        Tail is excluded because it moves.
        """
        head = snake[0]
        tail = snake[-1]

        blocked = set(obstacles) | set(snake[:-1])  # tail excluded
        visited = set()
        q = deque([head])

        while q:
            x, y = q.popleft()

            if (x, y) == tail:
                return True

            for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
                nx, ny = x + dx, y + dy
                nxt = (nx, ny)

                if (
                    0 <= nx < GRID_WIDTH and
                    0 <= ny < GRID_HEIGHT and
                    nxt not in blocked and
                    nxt not in visited
                ):
                    visited.add(nxt)
                    q.append(nxt)

        return False

    def decide_move(self, snake, food, obstacles):
        head = snake[0]

        blocked = set(obstacles) | set(snake[:-1])

        # 1️⃣ Try SAFE path to food
        path = self.astar(head, food, blocked)
        if path:
            next_pos = path[0]

            # simulate snake after eating food
            sim_snake = [next_pos] + snake[:-1]

            # tail must still be reachable
            if self.can_reach_tail(sim_snake, obstacles):
                return self.move_from_positions(head, next_pos)

        # 2️⃣ Try path to tail (stall safely)
        tail = snake[-1]
        path = self.astar(head, tail, blocked - {tail})
        if path:
            return self.move_from_positions(head, path[0])

        # 3️⃣ Hamiltonian fallback (guaranteed safe)
        head_i = self.index[head]
        next_pos = self.cycle[(head_i + 1) % self.cycle_len]
        return self.move_from_positions(head, next_pos)

    def astar(self, start, goal, blocked):
        from heapq import heappush, heappop

        h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])

        open_set = []
        heappush(open_set, (0 + h(start, goal), 0, start, []))
        visited = set()

        while open_set:
            _, cost, current, path = heappop(open_set)
            if current == goal:
                return path

            if current in visited:
                continue
            visited.add(current)

            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = current[0]+dx, current[1]+dy
                nxt = (nx, ny)

                if (
                    0 <= nx < GRID_WIDTH and
                    0 <= ny < GRID_HEIGHT and
                    nxt not in blocked
                ):
                    heappush(
                        open_set,
                        (cost+1+h(nxt, goal), cost+1, nxt, path+[nxt])
                    )
        return None

    def move_from_positions(self, a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        for k, v in self.moves.items():
            if v == (dx, dy):
                return k
        return None


def main():
    rclpy.init()
    node = SnakeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
