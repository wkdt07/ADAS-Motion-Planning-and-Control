import numpy as np
import math
import matplotlib.pyplot as plt
from map_3 import map

show_animation = True

class Node:
    def __init__(self, parent=None, position=None, is_reverse=False):
        self.parent = parent
        self.position = position         # [x, y, yaw]
        self.heading = position[2]
        self.g = 0
        self.h = 0
        self.f = 0
        self.is_reverse = is_reverse
        # 연속 후진 카운트
        self.reverse_count = (parent.reverse_count + 1) if (parent and is_reverse) else 0

def is_position_close(n1, n2, pos_threshold=0.3):
    dx = n1.position[0] - n2.position[0]
    dy = n1.position[1] - n2.position[1]
    return math.hypot(dx, dy) < pos_threshold

def is_yaw_close(n1, n2, yaw_threshold=0.2):
    dyaw = abs(n1.heading - n2.heading)
    dyaw = dyaw if dyaw < math.pi else 2*math.pi - dyaw
    return dyaw < yaw_threshold

def is_goal_reached(node, goal_node, pos_threshold=0.6, yaw_threshold=0.1):
    return is_position_close(node, goal_node, pos_threshold) and is_yaw_close(node, goal_node, yaw_threshold)

# 6가지 액션: 순행 좌/우/직진, 후진 좌/우/직진
def get_action(turning_radius, speed, dt):
    yaw_rate = speed / turning_radius
    travel_dist = speed * dt
    # [yaw_rate, dt, distance, is_reverse]
    return [
        [ yaw_rate,  dt,  travel_dist, False],   # 순행 좌회전
        [-yaw_rate,  dt,  travel_dist, False],   # 순행 우회전
        [      0.0,  dt,  travel_dist, False],   # 순행 직진
        [ yaw_rate,  dt, -travel_dist, True ],   # 후진 좌회전
        [-yaw_rate,  dt, -travel_dist, True ],   # 후진 우회전
        [      0.0,  dt, -travel_dist, True ]    # 후진 직진
    ]

def vehicle_move(parent_pos, yaw_rate, dt, speed):
    x0, y0, yaw0 = parent_pos
    if abs(yaw_rate) > 1e-5:
        R = speed / yaw_rate
        cx = x0 - R * math.sin(yaw0)
        cy = y0 + R * math.cos(yaw0)
        delta_yaw = yaw_rate * dt
        yaw1 = yaw0 + delta_yaw
        x1 = cx + R * math.sin(yaw1)
        y1 = cy - R * math.cos(yaw1)
    else:
        x1 = x0 + speed * dt * math.cos(yaw0)
        y1 = y0 + speed * dt * math.sin(yaw0)
        yaw1 = yaw0

    # yaw wrap
    yaw1 = (yaw1 + 2*math.pi) % (2*math.pi)
    return [x1, y1, yaw1]

def collision_check(parent_pos, yaw_rate, dt, obstacles, speed):
    num_checks = 10
    for i in range(1, num_checks+1):
        t = dt * (i/num_checks)
        x, y, _ = vehicle_move(parent_pos, yaw_rate, t, speed)
        for ox, oy, r in obstacles:
            if math.hypot(x-ox, y-oy) <= r:
                return True
    return False

def is_out_of_bounds(pos, space):
    x, y = pos[0], pos[1]
    xmin, xmax, ymin, ymax = space
    return not (xmin <= x <= xmax and ymin <= y <= ymax)

def calc_distance(n1, n2):
    dx = n1.position[0] - n2.position[0]
    dy = n1.position[1] - n2.position[1]
    return math.hypot(dx, dy)

def a_star(start, goal, space, obstacles,
           turning_radius=5.0, speed=2.0, dt=0.5,
           heur_weight=1.5, k_yaw=2.0, ramp_dist=5.0):

    # 2) closed_list 비교용 yaw 해상도 변경 ------------------------------
    def discret(n, yaw_res=30):      # ← 30° 해상도로 묶기
        x_b   = round(n.position[0],1)
        y_b   = round(n.position[1],1)
        yaw_b = round((n.heading*180/math.pi)/yaw_res)*yaw_res
        return (x_b, y_b, yaw_b, n.is_reverse)

    start_node = Node(None, start, is_reverse=False)
    goal_node  = Node(None, goal,  is_reverse=False)
    start_node.g = 0
    start_node.h = calc_distance(start_node, goal_node)
    start_node.f = start_node.g + heur_weight * start_node.h

    open_list, closed_list = [start_node], []

    while open_list:
        cur = min(open_list, key=lambda n: n.f)
        open_list.remove(cur)

        if is_goal_reached(cur, goal_node):
            path = []
            while cur:
                path.append(cur.position)
                cur = cur.parent
            return path[::-1]

        closed_list.append(cur)

        for yaw_rate, dt_i, dist, is_rev in get_action(turning_radius, speed, dt):
            child_pos = vehicle_move(cur.position, yaw_rate, dt_i, speed)
            if is_out_of_bounds(child_pos, space):        continue
            if collision_check(cur.position, yaw_rate, dt_i, obstacles, speed): continue

            child = Node(cur, child_pos, is_reverse=is_rev)
            child.g = cur.g + abs(dist) + (2.0 if cur.is_reverse != child.is_reverse else 0)

            # 3) 장애물 근처에서 heur_weight 강화 (dynamic weight) -------------
            #   예: 경계 margin 안쪽이면 가중치를 1.5배
            dist_to_goal = calc_distance(child, goal_node)
            near_obs = any(
                math.hypot(child.position[0]-ox, child.position[1]-oy) < (r + 0.5)
                for ox,oy,r in obstacles
            )
            w = heur_weight * (1.5 if near_obs else 1.0)
            # ----------------------------------------------------------------

            # 4) 거리 기반 램프 + yaw 오차 페널티 적용 -------------------------
            d_yaw  = child.heading - goal_node.heading
            yaw_err= abs(math.atan2(math.sin(d_yaw), math.cos(d_yaw)))
            ramp   = max(0.0, 1.0 - dist_to_goal / ramp_dist)
            child.h = dist_to_goal + k_yaw * ramp * yaw_err
            # ----------------------------------------------------------------

            child.f = child.g + w * child.h

            # 5) 중복 검사 & open_list 삽입 (discret 사용) --------------------
            disc_child = discret(child)
            if any(discret(n) == disc_child for n in closed_list):    continue
            better = any(discret(n)==disc_child and child.g>=n.g for n in open_list)
            if not better:
                open_list.append(child)
            # ----------------------------------------------------------------

        if show_animation:
            plt.plot(cur.position[0], cur.position[1], 'yo', alpha=0.3)
            if len(closed_list) % 100 == 0:
                plt.pause(0.01)

    return []

def main():
    start, goal, obstacles, space = map()

    # 1-1. 안전 여유분 포함해 장애물 반경을 늘려 줍니다.
    turning_radius = 5.0
    safety_margin = 0.5   # 0.5m 여유
    inflated_obs = [(ox, oy, r + turning_radius + safety_margin)
                    for (ox, oy, r) in obstacles]

    if show_animation:
        theta = np.linspace(0,2*math.pi,100)
        plt.figure(figsize=(8,8))
        plt.plot(start[0], start[1], 'bs', markersize=7); plt.text(start[0], start[1]+0.5, 'start')
        plt.plot(goal[0], goal[1], 'rs', markersize=7);   plt.text(goal[0], goal[1]+0.5, 'goal')
        # 1-2. 인플레이트된 장애물로 그리기
        for ox, oy, r in inflated_obs:
            plt.plot(ox + r*np.cos(theta), oy + r*np.sin(theta), 'k--', alpha=0.5)
        plt.axis(space); plt.grid(True)
        plt.xlabel("X [m]"); plt.ylabel("Y [m]"); plt.title("Hybrid A* with Reverse Curve")

    # 1-3. a_star 호출 시 obstacles → inflated_obs 로 교체
    path = a_star(start, goal, space, inflated_obs,
                  turning_radius=turning_radius, speed=2.0,
                  dt=0.5, heur_weight=1.5)
    print("Optimal path found!")
    if show_animation:
        path = np.array(path)
        if path.ndim == 2:
                plt.plot(path[:,0], path[:,1], "m.-")
        else:
            # 2차원이 아니면 플롯 건너뜀(디버그 메시지 출력)
            print(f"Cannot plot path, invalid shape: {path.shape}")
        plt.show()

if __name__ == "__main__":
    main()
