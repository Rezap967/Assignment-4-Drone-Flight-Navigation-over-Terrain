import heapq
import time

# ---------------------- GRID ----------------------
terrain = [
    ['S', '1', '1', '#', '1'],
    ['2', '3', '1', '9', '1'],
    ['3', '2', '2', '1', 'G']
]

rows, cols = len(terrain), len(terrain[0])

def find_pos(symbol):
    for r in range(rows):
        for c in range(cols):
            if terrain[r][c] == symbol:
                return (r, c)

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_cost(pos):
    val = terrain[pos[0]][pos[1]]
    if val in ['S', 'G']:
        return 1
    return int(val)

def reconstruct(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from.get(current)
        if current is None:
            return []
        path.append(current)
    path.reverse()
    return path

def astar_drone(start, goal):
    open_set = [(0 + manhattan(start, goal), 0, start)]
    came_from = {}
    g_score = {start: 0}
    nodes_explored = 0

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        nodes_explored += 1
        if current == goal:
            break
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = current[0]+dr, current[1]+dc
            if 0 <= nr < rows and 0 <= nc < cols and terrain[nr][nc] != '#':
                neighbor = (nr, nc)
                move_cost = get_cost(neighbor)
                tentative_g = g_score[current] + move_cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    f = tentative_g + manhattan(neighbor, goal)
                    heapq.heappush(open_set, (f, tentative_g, neighbor))
    return reconstruct(came_from, start, goal), nodes_explored

# ---------------------- RUN ----------------------
start = find_pos('S')
goal = find_pos('G')

start_time = time.time()
path, astar_nodes = astar_drone(start, goal)
astar_time = (time.time() - start_time) * 1000

# Placeholder GBFS (not implemented)
gbfs_time = 0.0
gbfs_nodes = 0

# Visualisasi
vis_terrain = [row[:] for row in terrain]
for r, c in path:
    if vis_terrain[r][c] not in ('S', 'G'):
        vis_terrain[r][c] = '*'

print("Grid Path:")
for row in vis_terrain:
    print(' '.join(row))

print("\nComparing result (based on Time in millisecond)")
print("Assignment\tGBFS\t\tA star")
print(f"4\t\t{gbfs_time:.2f} ms\t{astar_time:.2f} ms\n")

print("Comparing result (based on number of nodes)")
print("Assignment\tGBFS\tA star")
print(f"4\t\t{gbfs_nodes}\t{astar_nodes}")
