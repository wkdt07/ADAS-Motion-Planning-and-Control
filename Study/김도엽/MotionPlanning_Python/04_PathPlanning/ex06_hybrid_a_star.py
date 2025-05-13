import numpy as np
import math
import matplotlib.pyplot as plt
import random
from map_3 import map

show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent      # 부모 노드
        self.position = position  # [x, y, yaw]
        self.g = 0.0              # 시작으로부터의 비용
        self.h = 0.0              # 휴리스틱 비용
        self.f = 0.0              # 총 비용

def isSamePosition(node_1, node_2, epsilon_position=0.01):
    dx = node_1.position[0] - node_2.position[0]
    dy = node_1.position[1] - node_2.position[1]
    return math.hypot(dx, dy) < epsilon_position

def isSameYaw(node_1, node_2, epsilon_yaw=0.1):
    dyaw = abs(node_1.position[2] - node_2.position[2]) % (2*math.pi)
    if dyaw > math.pi:
        dyaw = 2*math.pi - dyaw
    return dyaw < epsilon_yaw

def get_action(R, Vx, delta_time_step):
    yaw_rate = Vx / R
    distance_travel = Vx * delta_time_step
    return [
        [ yaw_rate,                    delta_time_step, distance_travel],
        [-yaw_rate,                    delta_time_step, distance_travel],
        [ yaw_rate / 2.0,              delta_time_step, distance_travel],
        [-yaw_rate / 2.0,              delta_time_step, distance_travel],
        [ 0.0,                         delta_time_step, distance_travel]
    ]

def vehicle_move(position_parent, yaw_rate, delta_time, Vx):
    x, y, yaw = position_parent
    if abs(yaw_rate) < 1e-6:
        # 직진
        x += Vx * math.cos(yaw) * delta_time
        y += Vx * math.sin(yaw) * delta_time
        yaw_child = yaw
    else:
        # 원운동
        R = Vx / yaw_rate
        delta_yaw = yaw_rate * delta_time
        x += R * (math.sin(yaw + delta_yaw) - math.sin(yaw))
        y -= R * (math.cos(yaw + delta_yaw) - math.cos(yaw))
        yaw_child = yaw + delta_yaw

    # yaw 범위 정규화 [0, 2π)
    yaw_child %= 2 * math.pi
    return [x, y, yaw_child]

def collision_check(position_parent, yaw_rate, delta_time_step, obstacle_list, Vx):
    # 궤적을 N 샘플링해서 충돌 검사
    N = 10
    dt = delta_time_step / N
    pos = position_parent.copy()
    for _ in range(N):
        pos = vehicle_move(pos, yaw_rate, dt, Vx)
        for (ox, oy, r) in obstacle_list:
            if math.hypot(pos[0] - ox, pos[1] - oy) <= r:
                return True
    return False

def isNotInSearchingSpace(position_child, space):
    x, y, _ = position_child
    min_x, max_x, min_y, max_y = space
    return not (min_x <= x <= max_x and min_y <= y <= max_y)

def heuristic(cur_node, goal_node):
    dx = cur_node.position[0] - goal_node.position[0]
    dy = cur_node.position[1] - goal_node.position[1]
    return math.hypot(dx, dy)

def reconstruct_path(node):
    path = []
    while node is not None:
        path.append(node.position)
        node = node.parent
    return path[::-1]

def a_star(start, goal, space, obstacle_list, R, Vx, delta_time_step, weight):
    start_node = Node(None, start)
    goal_node  = Node(None, goal)
    open_list = [start_node]
    closed_list = []

    while open_list:
        # f가 가장 작은 노드 선택
        cur_node = min(open_list, key=lambda n: n.f)
        open_list.remove(cur_node)
        closed_list.append(cur_node)

        # 목표 도달 체크
        if isSamePosition(cur_node, goal_node, epsilon_position=0.2):
            return reconstruct_path(cur_node)

        # 자식 후보 생성
        for yaw_rate, dt, dist in get_action(R, Vx, delta_time_step):
            new_pos = vehicle_move(cur_node.position, yaw_rate, dt, Vx)
            if isNotInSearchingSpace(new_pos, space):
                continue
            if collision_check(cur_node.position, yaw_rate, dt, obstacle_list, Vx):
                continue

            child = Node(cur_node, new_pos)
            child.g = cur_node.g + dist
            child.h = heuristic(child, goal_node)
            child.f = child.g + weight * child.h

            # closed_list 체크
            if any(isSamePosition(child, closed, 0.3) and isSameYaw(child, closed, 0.2) for closed in closed_list):
                continue
            # open_list 중복 체크: 이미 같은 위치·방향에 더 낮은 g가 있으면 skip
            skip = False
            for open_node in open_list:
                if isSamePosition(child, open_node, 0.3) and isSameYaw(child, open_node, 0.2):
                    if child.g >= open_node.g:
                        skip = True
                    else:
                        open_list.remove(open_node)
                    break
            if skip:
                continue

            open_list.append(child)

        # 애니메이션 업데이트
        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(closed_list) % 100 == 0:
                plt.pause(0.001)

    return None  # 경로 미발견

def main():
    start, goal, obstacle_list, space = map()

    if show_animation:
        theta_plot = np.linspace(0,1,101) * np.pi * 2
        plt.figure(figsize=(8,8))
        plt.plot(start[0], start[1], 'bs',  markersize=7)
        plt.text(start[0], start[1]+0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs',  markersize=7)
        plt.text(goal[0], goal[1]+0.5, 'goal', fontsize=12)
        for ox, oy, r in obstacle_list:
            xc = ox + r * np.cos(theta_plot)
            yc = oy + r * np.sin(theta_plot)
            plt.plot(xc, yc, 'k-')
        plt.axis(space)
        plt.grid(True)
        plt.xlabel("X [m]"); plt.ylabel("Y [m]")
        plt.title("Hybrid A* Path Planning", fontsize=20)

    opt_path = a_star(start, goal, space, obstacle_list,
                      R=5.0, Vx=2.0, delta_time_step=0.5, weight=1.1)
    if opt_path is None:
        print("경로를 찾을 수 없습니다.")
        return

    opt_path = np.array(opt_path)
    print("Optimal path found! Total waypoints:", len(opt_path))
    if show_animation:
        plt.plot(opt_path[:,0], opt_path[:,1], "m.-", linewidth=2)
        plt.show()

if __name__ == "__main__":
    main()
