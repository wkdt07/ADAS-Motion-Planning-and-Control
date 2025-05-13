import numpy as np
import math
import matplotlib.pyplot as plt
from map_1 import map  # map() -> (start, goal, omap)

show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.f = 0  # 누적 비용

    def __eq__(self, other):
        return self.position == other.position

def get_action():
    """
    8방향 액션 정의: dx, dy, cost
    상, 하, 좌, 우: 비용 1
    대각선: 비용 sqrt(2)
    """
    cost_straight = 1.0
    cost_diag = math.sqrt(2)
    return [
        ( 0,  1, cost_straight),  # 상
        ( 0, -1, cost_straight),  # 하
        (-1,  0, cost_straight),  # 좌
        ( 1,  0, cost_straight),  # 우
        (-1, -1, cost_diag),      # 대각선
        (-1,  1, cost_diag),
        ( 1, -1, cost_diag),
        ( 1,  1, cost_diag),
    ]

def collision_check(omap, node):
    """
    맵의 장애물 좌표 리스트 omap을 돌면서 충돌 여부 확인
    omap[0], omap[1]에 각각 x, y 좌표들이 들어있다고 가정
    """
    for ox, oy in zip(omap[0], omap[1]):
        if node.position[0] == ox and node.position[1] == oy:
            return True
    return False

def dijkstra(start, goal, omap):
    # 시작/목표 노드 생성
    start_node = Node(None, start)
    goal_node = Node(None, goal)

    open_list = [start_node]
    closed_list = []

    while open_list:
        # 열린 리스트 중 f(비용) 최솟값 노드 선택
        current = min(open_list, key=lambda n: n.f)
        open_list.remove(current)
        closed_list.append(current)

        # 목표 도착하면 경로 재구성
        if current == goal_node:
            path = []
            node = current
            while node is not None:
                path.append(node.position)
                node = node.parent
            return path[::-1]  # 뒤집어서 반환

        # 자식 노드(후보) 생성
        for dx, dy, cost in get_action():
            new_pos = (current.position[0] + dx, current.position[1] + dy)
            child = Node(current, new_pos)
            child.f = current.f + cost

            # 충돌 체크
            if collision_check(omap, child):
                continue

            # 이미 닫힌 리스트에 있으면 무시
            if any(child == closed for closed in closed_list):
                continue

            # 열린 리스트에 있으면 비용 갱신 체크
            in_open = False
            for open_node in open_list:
                if child == open_node:
                    in_open = True
                    if child.f < open_node.f:
                        open_node.f = child.f
                        open_node.parent = current
                    break

            if not in_open:
                open_list.append(child)

        # 애니메이션: 탐색 현황 실시간 표시
        if show_animation:
            plt.plot(current.position[0], current.position[1], 'yo', alpha=0.3)
            if len(closed_list) % 100 == 0:
                plt.pause(0.001)

    # 경로를 찾지 못했을 때
    return None

def main():
    start, goal, omap = map()

    if show_animation:
        plt.figure(figsize=(8,8))
        # 시작·목표·장애물 그리기
        plt.plot(start[0], start[1], 'bs',  markersize=7); plt.text(start[0], start[1]+0.5, 'start')
        plt.plot(goal[0], goal[1], 'rs',  markersize=7); plt.text(goal[0], goal[1]+0.5, 'goal')
        plt.plot(omap[0], omap[1], '.k',  markersize=10)
        plt.grid(True); plt.axis("equal")
        plt.xlabel("X [m]"); plt.ylabel("Y [m]")
        plt.title("Dijkstra Algorithm")

    opt_path = dijkstra(start, goal, omap)
    if opt_path is None:
        print("경로를 찾지 못했습니다.")
        return

    print("Optimal path found!")
    opt_path = np.array(opt_path)

    if show_animation:
        plt.plot(opt_path[:,0], opt_path[:,1], "m.-", linewidth=2, markersize=5)
        plt.show()

if __name__ == "__main__":
    main()
