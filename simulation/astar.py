import numpy as np
import heapq

# Useful class for keeping track of the nodes being added
class Node:
    def __init__(self, position, parent=None):
        self.position = position  # Tuple (x, y)
        self.parent = parent
        self.g = 0  # Cost from start to current node. Assume every cost is +1
        self.h = 0  # Heuristic cost from current node to goal
        self.f = 0  # Total cost (g + h). Key to use for the queue.

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

# A* class for implementation
class AStar:
    # DO NOT modify this code
    def __init__(self):
        self.robot_start = (1, 28)
        self.goal = (20, 10)
        self.map_size = (30, 30)

        self.map_resolution = 0.05 # meters

    # Dont NOT modify this code
    def build_map(self,):
        # Initialize a 30x30 map with all cells set to 0 (free space)
        # NOTE: the resolution of the map of one cell = 0.05m
        # NOTE: the index (0,0) is in the top left corner went visualized using PyGame
        occupancy_grid = np.zeros(self.map_size, dtype=int)

        # Optionally, add some obstacles (set specific cells to 100)
        #Add a vertical wall in the middle of the map
        occupancy_grid[7:15, 12] = 100

        # Add a horizontal wall
        occupancy_grid[12, 7:15] = 100

        # Add a small obstacle
        occupancy_grid[10:20, 7:8] = 100
        occupancy_grid[7:9, 20:28] = 100
        occupancy_grid[15:17, 20:28] = 100

        return occupancy_grid

    # Do NOT modify this code
    def visualize_map(self, occupancy_grid, path=None):
        import pygame

        # Initialize Pygame
        pygame.init()

        # Define map dimensions and cell size
        map_size = (occupancy_grid.shape[0], occupancy_grid.shape[1])
        cell_size = 8  # Size of each cell in pixels
        width, height = map_size[0] * cell_size, map_size[1] * cell_size

        # Create a Pygame window
        screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("2D Map Visualization")

        # Define colors
        WHITE = (255, 255, 255)
        BLACK = (0, 0, 0)
        GRAY = (200, 200, 200)
        GREEN = (0, 255, 0)
        BLUE = (0, 0, 255)
        RED = (255, 0, 0)

        # Main loop
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Clear the screen
            screen.fill(GRAY)

            # Draw the grid
            for y in range(map_size[1]):
                for x in range(map_size[0]):
                    if occupancy_grid[y, x] == 100:
                        color = BLACK  # Obstacle
                    else:
                        color = WHITE  # Free space

                    # Draw the cell
                    pygame.draw.rect(
                        screen,
                        color,
                        (x * cell_size, y * cell_size, cell_size, cell_size),
                    )
            
            # Draw START location
            pygame.draw.rect(
                screen,
                GREEN,
                (self.robot_start[0] * cell_size, self.robot_start[1] * cell_size, cell_size, cell_size),
            )

            # Draw GOAL location
            pygame.draw.rect(
                screen,
                BLUE,
                (self.goal[0] * cell_size, self.goal[1] * cell_size, cell_size, cell_size),
            )

            # Draw grid lines
            for x in range(0, width, cell_size):
                pygame.draw.line(screen, BLACK, (x, 0), (x, height))
            for y in range(0, height, cell_size):
                pygame.draw.line(screen, BLACK, (0, y), (width, y))

            if path:
                for i in range(len(path)):
                    pygame.draw.rect(
                        screen,
                        RED,
                        (path[i][0] * cell_size, path[i][1] * cell_size, cell_size, cell_size),
                    )


            # Update the display
            pygame.display.flip()

        # Quit Pygame
        pygame.quit()

    def neighbor_pos(self, node):
        x, y = node.position
        return [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

    def heuristic(self, pos):
        # Implement your straight line heursitic here
        return abs(self.goal[0] - pos[0]) + abs(self.goal[1] - pos[1])
    
    def valid(self, pos, occupancy_grid):
        x, y = pos
        # in bounds
        if (x < 0) or (y < 0) or (x >= self.map_size[0]) or (y >= self.map_size[1]):
            return False
        
        # occupid
        if occupancy_grid[y,x] == 100:   # row then col I guess
            return False
        return True
    
    def reconstruct_path(self, node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return list(reversed(path))

    def astar(self, occupancy_grid):
        # Implement your A* search here
        open_list = []
        closed_list = set()
        
        # init dat shi
        start = Node(self.robot_start)
        start.g = 0
        start.h = self.heuristic(start.position)
        start.f = start.g + start.h
        heapq.heappush(open_list, start)

        while open_list:
            # pop the neighbors of the start, check if valid
            front = heapq.heappop(open_list)

            if front.position == self.goal:
                return self.reconstruct_path(front)
            # calculate cost
            neighbor_list_pos = self.neighbor_pos(front)
            

            for pos in neighbor_list_pos:
                if pos in closed_list:
                    continue
                if self.valid(pos, occupancy_grid):
                    neighbor = Node(pos, parent=front)
                    neighbor.g = front.g + 1
                    neighbor.h = self.heuristic(pos)
                    neighbor.f = neighbor.g + neighbor.h

                if self.add_to_open(open_list, neighbor):
                    heapq.heappush(open_list, neighbor)

            closed_list.add(front.position)
        return None

    def add_to_open(self, open_list, neighbor):
        # Check if a neighbor should be added to the open list.

        for node in open_list:
            if node.position == neighbor.position and node.f <= neighbor.f:
                return False
        return True

# Example usage
if __name__ == "__main__":
    # Example occupancy grid (0 = free, 100 = occupied)
    astar_instance = AStar()
    occupancy_grid = astar_instance.build_map()
    
    # Print map before search
    # Close the window before you can move onto the next part of the code
    astar_instance.visualize_map(occupancy_grid=occupancy_grid)

    path = astar_instance.astar(occupancy_grid)
    if path:
        print("Path found:", path)
    else:
        print("No path found.")

    # Print the solution found for your robot
    astar_instance. visualize_map(
        occupancy_grid=occupancy_grid,
        path=path)
