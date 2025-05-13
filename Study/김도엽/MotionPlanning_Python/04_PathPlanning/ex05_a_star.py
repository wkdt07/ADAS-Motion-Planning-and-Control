import numpy as np
import math
import matplotlib.pyplot as plt
from map_1 import map

show_animation = True

## 휴리스틱 함수: 유클리드 거리
def heuristic(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])

# 노드 클래스
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# 이웃 노드 가져오기 (8방향), 장애물 제외
def get_neighbors(node, obstacle_map):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                  (-1, -1), (-1, 1), (1, -1), (1, 1)]
    ox, oy = obstacle_map
    obstacles = set(zip(ox, oy))
    neighbors = []

    for dx, dy in directions:
        nx, ny = node.position[0] + dx, node.position[1] + dy
        if 0 <= nx <= 60 and 0 <= ny <= 60 and (nx, ny) not in obstacles:
            neighbors.append((nx, ny))
    return neighbors

# Weighted A* 알고리즘 구현
def weighted_astar(start, goal, obstacle_map, weight=2.0):  # ← weight 파라미터 추가
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    open_list = [start_node]
    closed_list = []
    batch_closed = []
    iteration = 0

    while open_list:
        iteration += 1
        current = min(open_list, key=lambda n: n.f)
        open_list.remove(current)
        closed_list.append(current)
        batch_closed.append(current.position)

        if current == goal_node:
            path = []
            node = current
            while node:
                path.append(node.position)
                node = node.parent
            return path[::-1]

        for pos in get_neighbors(current, obstacle_map):
            neighbor = Node(current, pos)
            if neighbor in closed_list:
                continue

            tentative_g = current.g + heuristic(current.position, neighbor.position)
            exists = next((n for n in open_list if neighbor == n and tentative_g >= n.g), None)
            if exists:
                continue

            neighbor.g = tentative_g
            neighbor.h = heuristic(neighbor.position, goal_node.position)
            neighbor.f = neighbor.g + weight * neighbor.h  # ← weighted A*
            open_list.append(neighbor)

        if show_animation and iteration % 20 == 0 and batch_closed:
            xs, ys = zip(*batch_closed)
            plt.plot(xs, ys, '.y')
            plt.pause(0.001)
            batch_closed.clear()

    return None

if __name__ == '__main__':
    start, goal, omap = map()

    if show_animation:
        plt.ion()
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.plot(omap[0], omap[1], '.k', markersize=10)
        ax.plot(start[0], start[1], 'bs', markersize=7)
        ax.text(start[0], start[1] + 0.5, 'start', fontsize=12)
        ax.plot(goal[0], goal[1], 'rs', markersize=7)
        ax.text(goal[0], goal[1] + 0.5, 'goal', fontsize=12)

    # weight 파라미터 조정 가능
    path = weighted_astar(start, goal, omap, weight=100)

    if path is None:
        print('No path found.')
    else:
        print(f'Path found: {len(path)} nodes')
        px, py = zip(*path)
        plt.plot(px, py, '-r', linewidth=2)

    if show_animation:
        plt.grid(True)
        plt.ioff()
        plt.show()
