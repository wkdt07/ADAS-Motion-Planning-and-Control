import numpy as np
import math
import matplotlib.pyplot as plt
import heapq
from map_3 import map

show_animation = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position  # [x, y, yaw]
        self.g = 0
        self.h = 0
        self.f = 0
    
    def __lt__(self, other):
        return self.f < other.f

def isSamePosition(node_1, node_2, epsilon_position=1.2):
    dist = np.linalg.norm(np.array(node_1.position[:2]) - np.array(node_2.position[:2]))
    return dist < epsilon_position

def isSameYaw(node_1, node_2, epsilon_yaw=0.25):
    diff = abs(node_1.position[2] - node_2.position[2]) % (2 * np.pi)
    if diff > np.pi:
        diff = 2 * np.pi - diff
    return diff < epsilon_yaw

def get_action(R, Vx, delta_time_step):
    yaw_rate = Vx / R
    distance_travel = Vx * delta_time_step
    return [
        [yaw_rate, delta_time_step, distance_travel],
        [-yaw_rate, delta_time_step, distance_travel],
        [yaw_rate / 2, delta_time_step, distance_travel],
        [-yaw_rate / 2, delta_time_step, distance_travel],
        [0.0, delta_time_step, distance_travel]
    ]

def vehicle_move(position_parent, yaw_rate, delta_time, Vx):
    x, y, yaw = position_parent
    if abs(yaw_rate) > 1e-6:
        R = Vx / yaw_rate
        cx = x - math.sin(yaw) * R
        cy = y + math.cos(yaw) * R
        dtheta = yaw_rate * delta_time
        x_new = cx + math.sin(yaw + dtheta) * R
        y_new = cy - math.cos(yaw + dtheta) * R
        yaw_new = yaw + dtheta
    else:
        x_new = x + Vx * delta_time * math.cos(yaw)
        y_new = y + Vx * delta_time * math.sin(yaw)
        yaw_new = yaw
    return [x_new, y_new, yaw_new % (2 * np.pi)]

def collision_check(position, yaw_rate, delta_time_step, obstacle_list, Vx):
    pos = vehicle_move(position, yaw_rate, delta_time_step, Vx)
    for obs in obstacle_list:
        dx = pos[0] - obs[0]
        dy = pos[1] - obs[1]
        if np.hypot(dx, dy) <= obs[2]:
            return True
    return False

def isNotInSearchingSpace(position, space):
    x, y = position[0], position[1]
    return not (space[0] <= x <= space[1] and space[2] <= y <= space[3])

def heuristic(cur_node, goal_node):
    pos_dist = np.linalg.norm(np.array(cur_node.position[:2]) - np.array(goal_node.position[:2]))
    yaw_diff = abs(cur_node.position[2] - goal_node.position[2]) % (2 * np.pi)
    yaw_penalty = min(yaw_diff, 2 * np.pi - yaw_diff)
    return pos_dist + 0.5 * yaw_penalty

def quantize(pos, precision=0.5):
    return (round(pos[0]/precision)*precision, round(pos[1]/precision)*precision, round(pos[2],1))

def a_star(start, goal, space, obstacle_list, R, Vx, delta_time_step, weight):
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    open_list = []
    heapq.heappush(open_list, (start_node.f, start_node))
    visited = set()

    while open_list:
        _, cur_node = heapq.heappop(open_list)
        q_key = quantize(cur_node.position)
        if q_key in visited:
            continue
        visited.add(q_key)

        if isSamePosition(cur_node, goal_node) and isSameYaw(cur_node, goal_node):
            path = []
            while cur_node is not None:
                path.append(cur_node.position)
                cur_node = cur_node.parent
            return path[::-1]

        for action in get_action(R, Vx, delta_time_step):
            yaw_rate, dt, _ = action
            new_position = vehicle_move(cur_node.position, yaw_rate, dt, Vx)
            if isNotInSearchingSpace(new_position, space):
                continue
            if collision_check(cur_node.position, yaw_rate, dt, obstacle_list, Vx):
                continue
            new_node = Node(cur_node, new_position)
            new_node.g = cur_node.g + Vx * dt
            new_node.h = heuristic(new_node, goal_node)
            new_node.f = new_node.g + weight * new_node.h
            if quantize(new_node.position) in visited:
                continue
            heapq.heappush(open_list, (new_node.f, new_node))

        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(visited) % 100 == 0:
                plt.pause(0.001)

    return None

def main():
    start, goal, obstacle_list, space = map()

    if show_animation:
        theta_plot = np.linspace(0,1,101) * 2 * np.pi
        plt.figure(figsize=(8,8))
        plt.plot(start[0], start[1], 'bs', markersize=7)
        plt.text(start[0], start[1]+0.5, 'start', fontsize=12)
        plt.plot(goal[0], goal[1], 'rs', markersize=7)
        plt.text(goal[0], goal[1]+0.5, 'goal', fontsize=12)
        for obs in obstacle_list:
            x_obstacle = obs[0] + obs[2] * np.cos(theta_plot)
            y_obstacle = obs[1] + obs[2] * np.sin(theta_plot)
            plt.plot(x_obstacle, y_obstacle, 'k-')
        plt.axis(space)
        plt.grid(True)
        plt.xlabel("X [m]"), plt.ylabel("Y [m]")
        plt.title("Hybrid A* Algorithm", fontsize=20)

    path = a_star(start, goal, space, obstacle_list, R=5.0, Vx=2.0, delta_time_step=0.5, weight=1.3)
    if path is not None:
        print("✅ Optimal path found!")
        if show_animation:
            path = np.array(path)
            plt.plot(path[:,0], path[:,1], "m.-")
            plt.show()
    else:
        print("❌ Path not found.")

if __name__ == "__main__":
    main()
