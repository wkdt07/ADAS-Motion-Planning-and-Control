# 힙큐 쓰도록 다익스트라 수정


import numpy as np
import math
import matplotlib.pyplot as plt
import heapq
from map_1 import map

show_animation = True

class Node:
    def __init__(self, parent=None, position=None, f=0):
        self.parent = parent
        self.position = position
        self.f = f

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.position == other.position


def get_action():
    return [
        [0, 1, 1],    # 상
        [0, -1, 1],   # 하
        [-1, 0, 1],   # 좌
        [1, 0, 1],    # 우
        [-1, 1, math.sqrt(2)],   # 좌상
        [-1, -1, math.sqrt(2)],  # 좌하
        [1, 1, math.sqrt(2)],    # 우상
        [1, -1, math.sqrt(2)]    # 우하
    ]


def collision_check(omap, node):
    x, y = node.position
    for ox, oy in zip(omap[0], omap[1]):
        if x == ox and y == oy:
            return True
    return False


def dijkstra(start, goal, map_obstacle):
    start_node = Node(None, start, f=0)
    goal_node = Node(None, goal)

    open_list = []
    open_dict = {}
    closed_set = set()

    heapq.heappush(open_list, start_node)
    open_dict[start_node.position] = start_node.f

    while open_list:
        cur_node = heapq.heappop(open_list)

        if cur_node.position in closed_set:
            continue

        closed_set.add(cur_node.position)

        if cur_node == goal_node:
            path = []
            while cur_node:
                path.append(cur_node.position)
                cur_node = cur_node.parent
            return path[::-1]

        for dx, dy, cost in get_action():
            next_pos = (cur_node.position[0] + dx, cur_node.position[1] + dy)
            new_cost = cur_node.f + cost
            new_node = Node(cur_node, next_pos, f=new_cost)

            if collision_check(map_obstacle, new_node):
                continue
            if next_pos in closed_set:
                continue

            # open_list에 더 낮은 cost로 이미 있는 경우는 무시
            if next_pos in open_dict and open_dict[next_pos] <= new_cost:
                continue

            heapq.heappush(open_list, new_node)
            open_dict[next_pos] = new_cost

        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(closed_set) % 100 == 0:
                plt.pause(0.1)

    return []


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
        plt.title("Dijkstra algorithm", fontsize=20)

    opt_path = dijkstra(start, goal, omap)
    print("Optimal path found!")
    opt_path = np.array(opt_path)

    if show_animation:
        plt.plot(opt_path[:, 0], opt_path[:, 1], "m.-")
        plt.show()


if __name__ == "__main__":
    main()


'''
import numpy as np
import math
import matplotlib.pyplot as plt
import random
from map_1 import map

show_animation  = True

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.f = 0

    def __eq__(self, other):
        if self.position == other.position:
            return True
                
def get_action():
    # action = [dx, dy, cost]
    # action_set = [action1, action2, ...]
    # dx, dy, 이동비용
    action_set = [
        [0, 1, 1],    # 상
        [0, -1, 1],   # 하
        [-1, 0, 1],   # 좌
        [1, 0, 1],    # 우
        [-1, 1, math.sqrt(2)],   # 좌상
        [-1, -1, math.sqrt(2)],  # 좌하
        [1, 1, math.sqrt(2)],    # 우상
        [1, -1, math.sqrt(2)]    # 우하
    ]
    return action_set

def collision_check(omap, node):
    # Check if node position == obstacle position
    x, y = node.position
    for ox, oy in zip(omap[0], omap[1]):
        if x == ox and y == oy:
            return True
    return False

def dijkstra(start, goal, map_obstacle):
    start_node = Node(None, start)
    goal_node = Node(None, goal)
    
    open_list = [start_node]
    closed_list = []

    while open_list:
        # 최소 비용 노드 선택
        cur_node = min(open_list, key=lambda node: node.f)
        open_list.remove(cur_node)
        closed_list.append(cur_node)
        # Find node with lowest cost
                
        # If goal, return optimal path
        
        # If not goal, move from open list to closed list

        # Generate child candidate
        if cur_node == goal_node:
            path = []
            while cur_node is not None:
                path.append(cur_node.position)
                cur_node = cur_node.parent
            return path[::-1]  # 역순 반환

        # 자식 노드 생성
        action_set = get_action()
        for dx, dy, cost in action_set:
             # If collision expected, do nothing

            # If not collision, create child node

            # If already in closed list, do nothing

            # If not in closed list, update open list
            node_pos = (cur_node.position[0] + dx, cur_node.position[1] + dy)
            new_node = Node(cur_node, node_pos)
            new_node.f = cur_node.f + cost

            if collision_check(map_obstacle, new_node):
                continue

            if any((child.position == new_node.position) for child in closed_list):
                continue

            # open_list에 더 효율적인 노드가 있으면 추가 안 함
            for open_node in open_list:
                if new_node == open_node and new_node.f >= open_node.f:
                    break
            else:
                open_list.append(new_node)

        # 시각화
        if show_animation:
            plt.plot(cur_node.position[0], cur_node.position[1], 'yo', alpha=0.5)
            if len(closed_list) % 100 == 0:
                plt.pause(0.01)

    return []

                

def main():

    start, goal, omap = map()

    if show_animation == True:
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
    print("Optimal path found!")
    opt_path = np.array(opt_path)
    if show_animation == True:
        plt.plot(opt_path[:,0], opt_path[:,1], "m.-")
        plt.show()


if __name__ == "__main__":
    main()
'''
    

