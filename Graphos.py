
import osmnx as ox
import networkx as nx
import heapq
import matplotlib.pyplot as plt

def heuristic(a, b, data):
    return ((data.nodes[a]['x'] - data.nodes[b]['x'])**2 + (data.nodes[a]['y'] - data.nodes[b]['y'])**2)**0.5

def astar_algorithm(graph, start, goal):
    open_heap = []
    heapq.heappush(open_heap, (0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while open_heap:
        _, current = heapq.heappop(open_heap)
        
        if current == goal:
            break
        
        for neighbor in graph.neighbors(current):
            edge_data = graph.get_edge_data(current, neighbor)
            if edge_data is None:
                continue
            edge_length = edge_data.get('length', float('inf'))
            new_cost = cost_so_far[current] + edge_length
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(current, neighbor, graph)
                heapq.heappush(open_heap, (priority, neighbor))
                came_from[neighbor] = current
    
    return came_from, cost_so_far

def plot_route(graph, came_from, start, goal):
    route = []
    current = goal
    while current != start:
        route.append(current)
        current = came_from.get(current)
    route.append(start)
    route.reverse()
    
    # Plot configuration
    fig, ax = ox.plot_graph_route(graph, route, route_color='red', route_linewidth=6, node_size=0, bgcolor='k', edge_linewidth=1.5, edge_color='#BCBCCB')
    plt.tight_layout()
    plt.show()

# Map configuration
place_name = "Bogotá, Colombia"
graph = ox.graph_from_place(place_name, network_type='drive')
graph = ox.add_edge_speeds(graph)
graph = ox.add_edge_travel_times(graph)
graph = ox.add_edge_bearings(graph)

# My data
lat_casa, lon_casa = 4.74936534789802, -74.02734537137499
lat_trabajo, lon_trabajo = 4.574216141758772, -74.13322722009951

# Start and end points
start_point = ox.distance.nearest_nodes(graph, lon_casa, lat_casa)
end_point = ox.distance.nearest_nodes(graph, lon_trabajo, lat_trabajo)
came_from, cost = astar_algorithm(graph, start_point, end_point)
print(f"Path from {start_point} to {end_point} calculated.")
plot_route(graph, came_from, start_point, end_point)

