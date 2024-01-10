import time

import matplotlib.pyplot as plt
from mazelib import Maze
from mazelib.generate.DungeonRooms import DungeonRooms


def showPNG(grid, start=None, end=None, shortest_path=None, visited_nodes=None):
    """Generate a simple image of the maze."""
    plt.figure(figsize=(10, 5))
    plt.imshow(grid, cmap=plt.cm.binary, interpolation='nearest')
    plt.xticks([]), plt.yticks([])
    # plot start and end
    if start is not None:
        plt.plot(start[1], start[0], 'o', markersize=10, color='green')
    if end is not None:
        plt.plot(end[1], end[0], 'o', markersize=10, color='red')
    # plot the shortest path
    if shortest_path is not None:
        plt.plot([p[1] for p in shortest_path], [p[0] for p in shortest_path], linewidth=3, color='cyan')
    # add to plot visited nodes with opacity
    if visited_nodes is not None:
        for node in visited_nodes:
            plt.plot(node[1], node[0], 'o', markersize=5, color='blue', alpha=0.1)
    plt.show()


import heapq


class MazeSolver:
    def __init__(self, grid, start, end):
        self.grid = grid
        self.start = start
        self.end = end
        self.solved = False
        self.visited_nodes = set()
        self.path = None

    def get_heuristic_distance_to_goal(self, point):
        """Calculate the heuristic distance from a point to the end point using Manhattan distance"""
        return abs(point[0] - self.end[0]) + abs(point[1] - self.end[1])

    def state_is_valid(self, x, y):
        """Check if a state is not in a wall and is within the grid"""
        return 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]) and self.grid[x][y] == 0

    def find_neighbors(self, point: tuple[int, int]):
        """Find all the valid neighbors of a state"""
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = point[0] + dx, point[1] + dy
            if self.state_is_valid(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

    def astar(self, start, goal):
        open_set = []

        # Priority queue with (F-score, node)
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}  # Cost from start along best known path
        visited_nodes = {start}

        while open_set:
            _, current = heapq.heappop(open_set)
            print("current", current)
            visited_nodes.add(current)

            if current == goal:
                # Reconstruct the path and return
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                self.solved = True
                self.visited_nodes = visited_nodes
                self.path = path
                return path

            valid_neighbor_states: list[tuple] = self.find_neighbors(current)

            for neighbor in valid_neighbor_states:
                # f(n)=g(n)+h(n)
                # g(n) is the cost of the path from the start node to n
                # h(n) is a heuristic that estimates the cost of the cheapest path from n to the goal

                # Assuming uniform cost for each step
                tentative_g = g_score[current] + 1

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # This path is the most promising according to g_score
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.get_heuristic_distance_to_goal(neighbor)
                    heapq.heappush(open_set, (f_score, neighbor))
                    came_from[neighbor] = current

        self.solved = True
        self.visited_nodes = visited_nodes
        return None  # No path found


if __name__ == '__main__':
    # Maze.set_seed(123)

    m = Maze()
    m.generator = DungeonRooms(50, 50, rooms=[[(1, 1), (2, 4)]], )
    m.generate()
    m.generate_entrances()

    # make sure start and end are reachable
    m.grid[m.start] = 0
    m.grid[m.end] = 0

    print("start", m.start)
    print("end", m.end)

    # time solving
    start = time.time()
    solver = MazeSolver(m.grid, m.start, m.end)
    path = solver.astar(m.start, m.end)
    end = time.time()
    # print nicely time
    print(f"time solving: : {(end - start) * 1000:.4f} ms")
    # path = astar(m.grid, m.start, m.end)
    # print("Shortest path:", path)
    # print("Visited nodes:", solver.visited_nodes)
    showPNG(m.grid, m.start, m.end, path, visited_nodes=solver.visited_nodes)
