import numpy as np
import math
import matplotlib.pyplot as plt
from map_2 import map

show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def get_action():
    # 8방향 이동 (대각선 포함)
    action_set = [
        (0, 1, 1),   # ↑
        (0, -1, 1),  # ↓
        (-1, 0, 1),  # ←
        (1, 0, 1),   # →
        (-1, 1, math.sqrt(2)),   # ↖
        (1, 1, math.sqrt(2)),    # ↗
        (-1, -1, math.sqrt(2)),  # ↙
        (1, -1, math.sqrt(2))    # ↘
    ]
    return action_set

def heuristic(a, b):
    # 유클리드 거리
    return math.hypot(b[0] - a[0], b[1] - a[1])

def collision_check(omap, node):
    x, y = node.position
    ox, oy = omap
    if x < 0 or x > 60 or y < 0 or y > 60:
        return True
    for ox_i, oy_i in zip(ox, oy):
        if node.position == (ox_i, oy_i):
            return True
    return False

def a_star(start, goal, map_obstacle):
    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_list = []
    closed_list = []

    open_list.append(start_node)

    while open_list:
        # f 값이 가장 작은 노드 선택
        cur_node = min(open_list, key=lambda o: o.f)
        open_list.remove(cur_node)
        closed_list.append(cur_node)

        # 목표 도달
        if cur_node == goal_node:
            path = []
            while cur_node is not None:
                path.append(cur_node.position)
                cur_node = cur_node.parent
            return path[::-1]  # 경로 반환

        for dx, dy, cost in get_action():
            new_pos = (cur_node.position[0] + dx, cur_node.position[1] + dy)
            new_node = Node(cur_node, new_pos)

            if collision_check(map_obstacle, new_node):
                continue

            if new_node in closed_list:
                continue

            new_node.g = cur_node.g + cost
            new_node.h = heuristic(new_node.position, goal_node.position)
            new_node.f = new_node.g + new_node.h

            # open_list에 있는 동일한 노드보다 더 나은 경로이면 갱신
            skip = False
            for node in open_list:
                if new_node == node and new_node.g >= node.g:
                    skip = True
                    break
            if not skip:
                open_list.append(new_node)

        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'go', alpha=1.0)
            if len(closed_list) % 100 == 0:
                plt.pause(0.01)

    return None  # 경로 없음

def main():
    start, goal, omap = map()

    if show_animation:
        plt.figure(figsize=(8, 8))
        plt.plot(start[0], start[1], 'bs', markersize=7)
        plt.text(start[0], start[1] + 0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs', markersize=7)
        plt.text(goal[0], goal[1] + 0.5, 'goal', fontsize=12)
        plt.plot(omap[0], omap[1], '.k', markersize=10)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("A* Algorithm", fontsize=20)

    opt_path = a_star(start, goal, omap)
    if opt_path is None:
        print("경로를 찾을 수 없습니다.")
        return
    print("경로 탐색 완료!")
    opt_path = np.array(opt_path)

    if show_animation:
        plt.plot(opt_path[:, 0], opt_path[:, 1], "r.-", linewidth=2)
        plt.show()

if __name__ == "__main__":
    main()
