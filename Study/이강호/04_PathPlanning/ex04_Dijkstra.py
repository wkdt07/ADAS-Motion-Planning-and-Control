import numpy as np
import math
import matplotlib.pyplot as plt
from map_1 import map

show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.f = 0  # 다익스트라는 g만 사용, f=g로 간주

    def __eq__(self, other):
        return self.position == other.position

def get_action():
    # 8방향 이동: 상, 하, 좌, 우, 좌상, 우상, 좌하, 우하
    action_set = [
        (0, 1, 1),    # 위
        (0, -1, 1),   # 아래
        (-1, 0, 1),   # 왼쪽
        (1, 0, 1),    # 오른쪽
        (-1, 1, math.sqrt(2)),   # 좌상
        (1, 1, math.sqrt(2)),    # 우상
        (-1, -1, math.sqrt(2)),  # 좌하
        (1, -1, math.sqrt(2))    # 우하
    ]
    return action_set


def collision_check(omap, node):
    x, y = node.position
    ox, oy = omap
    if x < 0 or x > 60 or y < 0 or y > 60:
        return True
    for ox_i, oy_i in zip(ox, oy):
        if node.position == (ox_i, oy_i):
            return True
    return False

def dijkstra(start, goal, map_obstacle):
    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_list = []
    closed_list = []

    open_list.append(start_node)

    while open_list:
        # 비용이 가장 작은 노드 선택
        cur_node = min(open_list, key=lambda o: o.f)
        open_list.remove(cur_node)
        closed_list.append(cur_node)

        # 목표 도달 시 경로 생성
        if cur_node == goal_node:
            path = []
            while cur_node is not None:
                path.append(cur_node.position)
                cur_node = cur_node.parent
            return path[::-1]  # 역순 반환

        # 인접 노드 탐색
        for dx, dy, cost in get_action():
            new_pos = (cur_node.position[0] + dx, cur_node.position[1] + dy)
            new_node = Node(cur_node, new_pos)
            new_node.f = cur_node.f + cost

            if collision_check(map_obstacle, new_node):
                continue

            if new_node in closed_list:
                continue

            # 더 나은 경로면 open list에 추가
            in_open = False
            for node in open_list:
                if new_node == node and new_node.f >= node.f:
                    in_open = True
                    break
            if not in_open:
                open_list.append(new_node)

        # 시각화
        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.3)
            if len(closed_list) % 100 == 0:
                plt.pause(0.01)

    return None  # 경로 없음

def main():
    start, goal, omap = map()

    if show_animation:
        plt.figure(figsize=(8,8))
        plt.plot(start[0], start[1], 'bs',  markersize=7)
        plt.text(start[0], start[1]+0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs',  markersize=7)
        plt.text(goal[0], goal[1]+0.5, 'goal', fontsize=12)
        plt.plot(omap[0], omap[1], '.k',  markersize=10)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("Dijkstra algorithm", fontsize=20)

    opt_path = dijkstra(start, goal, omap)
    if opt_path is None:
        print("경로를 찾을 수 없습니다.")
        return
    print("Optimal path found!")
    opt_path = np.array(opt_path)

    if show_animation:
        plt.plot(opt_path[:, 0], opt_path[:, 1], "m.-", linewidth=2)
        plt.show()

if __name__ == "__main__":
    main()
