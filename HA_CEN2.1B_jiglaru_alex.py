import heapq

class TSP:
    def __init__(self, cities, distances):
        self.cities = cities
        self.distances = distances

    def get_neighbors(self, city): # (get neighboring cities and their distances from the current city)
        neighbors = []
        for i, distance in enumerate(self.distances[city]):
            if distance > 0:  
                neighbors.append((i, distance))
        return neighbors

    def path_cost(self, path): # (calculate the total cost of a given path)
        return sum(self.distances[path[i]][path[i + 1]] for i in range(len(path) - 1))

    def is_goal(self, path): # (check if the given path is a complete tour)
        return len(path) == len(self.cities) and path[0] == path[-1]

def dfs(tsp): # (Depth-First Search)
    stack = [(0, [0], 0)]  # (current city, path, cost)
    best_path = None
    best_cost = float('inf')
    
    while stack:
        current_city, path, cost = stack.pop()
        
        if len(path) == len(tsp.cities):
            path.append(0)
            cost += tsp.distances[current_city][0]
            if cost < best_cost:
                best_cost = cost
                best_path = path
            continue
        
        for neighbor, distance in tsp.get_neighbors(current_city):
            if neighbor not in path:
                stack.append((neighbor, path + [neighbor], cost + distance))
    
    return best_path, best_cost

def ucs(tsp): # (Uniform Cost Search)
    priority_queue = [(0, 0, [0])]  # (cost, current city, path)
    best_path = None
    best_cost = float('inf')
    
    while priority_queue:
        cost, current_city, path = heapq.heappop(priority_queue)
        
        if len(path) == len(tsp.cities):
            path.append(0)
            cost += tsp.distances[current_city][0]
            if cost < best_cost:
                best_cost = cost
                best_path = path
            continue
        
        for neighbor, distance in tsp.get_neighbors(current_city):
            if neighbor not in path:
                heapq.heappush(priority_queue, (cost + distance, neighbor, path + [neighbor]))
    
    return best_path, best_cost

def heuristic(current_city, remaining_cities, distances):
    # Simple heuristic: minimum distance to any remaining city
    return min(distances[current_city][city] for city in remaining_cities) if remaining_cities else 0

def a_star(tsp): # (A* Search algorithm)
    priority_queue = [(0, 0, [0])]  # (cost + heuristic, current city, path)
    best_path = None
    best_cost = float('inf')
    
    while priority_queue:
        f, current_cost, path = heapq.heappop(priority_queue)
        current_city = path[-1]
        
        if len(path) == len(tsp.cities):
            path.append(0)
            total_cost = current_cost + tsp.distances[current_city][0]
            if total_cost < best_cost:
                best_cost = total_cost
                best_path = path
            continue
        
        remaining_cities = set(range(len(tsp.cities))) - set(path)
        for neighbor, distance in tsp.get_neighbors(current_city):
            if neighbor not in path:
                g = current_cost + distance
                h = heuristic(neighbor, remaining_cities, tsp.distances)
                f = g + h
                heapq.heappush(priority_queue, (f, g, path + [neighbor]))
    
    return best_path, best_cost

if __name__ == "__main__":
    cities = ['A', 'B', 'C', 'D']
    distances = [
        [0, 10, 15, 20],
        [10, 0, 35, 25],
        [15, 35, 0, 30],
        [20, 25, 30, 0]
    ]
    
    tsp = TSP(cities, distances)
    
    dfs_path, dfs_cost = dfs(tsp)
    print("DFS Path:", dfs_path)
    print("DFS Cost:", dfs_cost)
    
    ucs_path, ucs_cost = ucs(tsp)
    print("UCS Path:", ucs_path)
    print("UCS Cost:", ucs_cost)
    
    a_star_path, a_star_cost = a_star(tsp)
    print("A* Path:", a_star_path)
    print("A* Cost:", a_star_cost)