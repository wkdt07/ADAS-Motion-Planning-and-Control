import numpy as np
import math
import matplotlib.pyplot as plt
from map_3 import map

# 후진-전진 전환 시 페널티
switch_penalty = 0.5

# 시각화 ON/OFF
show_animation = True

def isGoalReached(node, goal_node, pos_th=0.3, yaw_th=0.2):
    """
    위치와 헤딩이 goal threshold 내에 있는지 확인
    """
    # Position check
    dx = node.position[0] - goal_node.position[0]
    dy = node.position[1] - goal_node.position[1]
    dpos = math.hypot(dx, dy)

    # Yaw check (wrap-around 고려)
    dyaw = abs(node.heading - goal_node.heading)
    dyaw = dyaw if dyaw < math.pi else 2 * math.pi - dyaw

    return (dpos < pos_th) and (dyaw < yaw_th)

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position  # [x, y, yaw]
        self.heading = position[2]
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0
        self.is_reverse = False
        self.reverse_count = 0


def get_action(R, Vx, delta_time_step, reverse_penalty=1.5):
    """
    (yaw_rate, dt, travel_dist, cost_multiplier, is_reverse_flag)
    """
    base_dist = Vx * delta_time_step
    yaw_rate = Vx / R
    actions = []

    # 전진 동작 (cost_multiplier=1)
    for rate in [yaw_rate, -yaw_rate, yaw_rate/2, -yaw_rate/2, 0.0]:
        actions.append([rate, delta_time_step,  base_dist, 1.0, False])
    # 후진 동작 (cost_multiplier=reverse_penalty)
    for rate in [yaw_rate, -yaw_rate, yaw_rate/2, -yaw_rate/2, 0.0]:
        actions.append([rate, delta_time_step, -base_dist, reverse_penalty, True])

    return actions


def vehicle_move(position_parent, yaw_rate, delta_time, travel_distance):
    x, y, yaw = position_parent
    if abs(yaw_rate) > 1e-5:
        R = travel_distance / yaw_rate
        cx = x - R * math.sin(yaw)
        cy = y + R * math.cos(yaw)
        delta_yaw = yaw_rate * delta_time
        yaw_child = yaw + delta_yaw
        x_child = cx + R * math.sin(yaw_child)
        y_child = cy - R * math.cos(yaw_child)
    else:
        x_child = x + travel_distance * math.cos(yaw)
        y_child = y + travel_distance * math.sin(yaw)
        yaw_child = yaw

    # wrap yaw to [0, 2pi)
    yaw_child %= 2 * math.pi
    return [x_child, y_child, yaw_child]


def collision_check(position_parent, yaw_rate, delta_time, obstacle_list, travel_dist):
    # travel_dist는 방향(전진/후진)을 포함한 이동 거리
    Vx = abs(travel_dist) / delta_time
    num_check = 10
    for i in range(1, num_check + 1):
        dt = delta_time * (i / num_check)
        pos = vehicle_move(position_parent, yaw_rate, dt, travel_dist)
        x, y = pos[0], pos[1]
        for obs in obstacle_list:
            dx = x - obs[0]
            dy = y - obs[1]
            if math.hypot(dx, dy) <= obs[2]:
                return True
    return False


def isNotInSearchingSpace(position, space):
    x, y = position[0], position[1]
    x_min, x_max, y_min, y_max = space
    return not (x_min <= x <= x_max and y_min <= y <= y_max)


def heuristic(cur_node, goal_node):
    # 거리 + 목표 근처에서 헤딩 페널티 강화
    dx = cur_node.position[0] - goal_node.position[0]
    dy = cur_node.position[1] - goal_node.position[1]
    dpos = math.hypot(dx, dy)

    dyaw = abs(cur_node.heading - goal_node.heading)
    dyaw = dyaw if dyaw < math.pi else 2 * math.pi - dyaw

    # 목표 근처에서 yaw 영향력 증가
    heading_factor = min(1.0, 10.0 / (dpos + 1e-3))
    return dpos + 2.0 * dyaw * heading_factor


def a_star(start, goal_input, space, obstacle_list, R, Vx, delta_time_step, weight):
    # start: [x, y, yaw]
    # goal_input: [x, y, yaw]
    start_node = Node(None, start)
    start_node.h = heuristic(start_node, Node(None, goal_input))
    start_node.f = start_node.g + weight * start_node.h

    goal_node = Node(None, goal_input)

    open_list = [start_node]
    closed_list = []

    while open_list:
        # f-value가 가장 작은 노드 선택
        cur = min(open_list, key=lambda n: n.f)
        open_list.remove(cur)

        # Goal 확인 (position + heading)
        if isGoalReached(cur, goal_node):
            path = []
            node = cur
            while node:
                path.append(node.position)
                node = node.parent
            return path[::-1]

        closed_list.append(cur)

        # 자식 노드 확장
        for yaw_rate, dt, travel_dist, penalty, is_rev in get_action(R, Vx, delta_time_step):
            pos_child = vehicle_move(cur.position, yaw_rate, dt, travel_dist)

            if isNotInSearchingSpace(pos_child, space):
                continue
            if collision_check(cur.position, yaw_rate, dt, obstacle_list, travel_dist):
                continue

            child = Node(cur, pos_child)
            child.is_reverse = is_rev
            child.reverse_count = cur.reverse_count + 1 if is_rev else 0

            # 비용 계산: 이동 거리 * 패널티 * (후진 연속 횟수 영향) + 전환 페널티
            cost = abs(travel_dist) * penalty * (1 + 0.1 * child.reverse_count)
            if cur.is_reverse != child.is_reverse:
                cost += switch_penalty

            child.g = cur.g + cost
            child.h = heuristic(child, goal_node)
            child.f = child.g + weight * child.h

            # 중복 검사 (closed)
            if any(isGoalReached(child, old) for old in closed_list):
                continue

            # open_list 내 기존 노드와 비교 후 skip
            better = False
            for old in open_list:
                if isGoalReached(child, old) and child.g >= old.g:
                    better = True
                    break
            if better:
                continue

            open_list.append(child)

        # 시각화
        if show_animation:
            plt.plot(cur.position[0], cur.position[1], 'yo', alpha=0.3)
            if len(closed_list) % 100 == 0:
                plt.pause(0.01)

    # 경로 미발견 시 빈 리스트 반환
    return []


def main():
    # map()에서 yaw까지 포함된 start, goal을 넘겨야 합니다.
    start, goal, obstacle_list, space = map()

    if show_animation:
        theta = np.linspace(0, 2*np.pi, 100)
        plt.figure(figsize=(8,8))
        plt.plot(start[0], start[1], 'bs', markersize=7); plt.text(start[0], start[1]+0.5, 'start')
        plt.plot(goal[0],  goal[1],  'rs', markersize=7); plt.text(goal[0], goal[1]+0.5,  'goal')
        for ox, oy, r in obstacle_list:
            plt.plot(ox + r*np.cos(theta), oy + r*np.sin(theta), 'k-')
        plt.axis(space); plt.grid(True);
        plt.xlabel('X'), plt.ylabel('Y')
        plt.title('Hybrid A* with Reverse Motion')

    path = a_star(start, goal, space, obstacle_list, R=5.0, Vx=2.0, delta_time_step=0.5, weight=2.5)
    if path:
        path = np.array(path)
        plt.plot(path[:,0], path[:,1], 'm.-'); plt.show()
    else:
        print('경로를 찾지 못했습니다.')

if __name__ == '__main__':
    main()
