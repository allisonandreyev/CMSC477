import heapq
import csv

def dijkstra(map, start, goal):
    priority_queue = [(0, start)]
    distances = {start: 0}
    parent = {start: None}
    
    while priority_queue:
        current_distance, current = heapq.heappop(priority_queue)

        if current == goal:
            return reconstruct_path(parent, start, goal)

        for neighbor in get_neighbors(current, map):
            if map[neighbor[0]][neighbor[1]] != -1:
                distance = current_distance + 1  # Assuming uniform cost

                if neighbor not in distances or distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))
                    parent[neighbor] = current
    return None  # No path found

def openFile(name):
    with open(name, newline='') as csvfile:
        reader = csv.reader(csvfile)
        return [[int(cell) for cell in row] for row in reader]

def find_start_goal(map):
    start = None
    goal = None
    for r in range(len(map)):
        for c in range(len(map[0])):
            if map[r][c] == 2:
                start = (r, c)
            elif map[r][c] == 3:
                goal = (r, c)
    return start, goal

def get_neighbors(pos, map):
    row, col = pos
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

    for dr, dc in directions:
        new_row, new_col = row + dr, col + dc
        if 0 <= new_row < len(map) and 0 <= new_col < len(map[0]):
            neighbors.append((new_row, new_col))
    return neighbors

def reconstruct_path(parent, start, goal):
    path = []
    while goal is not None:
        path.append(goal)
        goal = parent[goal]
    path.reverse()  # Reverse the path to start from the beginning
    return path

def main():
   
    map = [
        [0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,1,1,1,1,1,1,1,0,0,0],
        [0,0,0,1,0,0,0,0,0,1,0,0,0],
        [0,0,0,1,0,0,0,0,0,1,0,0,0],
        [0,2,0,1,0,0,0,0,0,1,0,3,0],
        [0,0,0,1,0,0,1,0,0,1,0,0,0],
        [0,0,0,0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,0,0,1,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0]
    ] 
    # Change filename as needed
    start, goal = find_start_goal(map)

    # # Run Dijkstra's
    dijkstra_path = dijkstra(map, start, goal)
    print("Dijkstra Path:", dijkstra_path)

if __name__ == "__main__":
    main()


