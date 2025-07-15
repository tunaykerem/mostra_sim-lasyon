import heapq
import numpy as np
def heuristic(a, b):
    # Manhattan mesafesi hesapla
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(matrix, start, goal):
    rows, cols = matrix.shape
    open_set = []
    closed_set = set()
    came_from = {}
    cost_so_far = {}

    heapq.heappush(open_set, (0, start))
    came_from[start] = None
    cost_so_far[start] = 0

    while open_set:
        current_cost, current_node = heapq.heappop(open_set)

        if current_node == goal:
            break

        if current_node in closed_set:
            continue

        closed_set.add(current_node)

        for i, j in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = current_node[0] + i, current_node[1] + j

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and matrix[neighbor] != 0:
                new_cost = cost_so_far[current_node] + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current_node

    path = reconstruct_path(came_from, start, goal)
    return cost_so_far[goal], path


def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path