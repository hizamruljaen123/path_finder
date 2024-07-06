import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import PriorityQueue
import os

# Function to download or load the street network graph for Jakarta
def load_or_download_graph(place_name):
    filename = f"{place_name}_graph.graphml"

    # Check if the graph file exists
    if os.path.exists(filename):
        # Load the graph from file
        G = ox.load_graphml(filename)
    else:
        # Download and save the graph
        G = ox.graph_from_place(place_name, network_type='drive')
        ox.save_graphml(G, filename)

    return G

# Define the place name
place_name = "Jakarta, Indonesia"

# Load or download the street network graph for Jakarta
G = load_or_download_graph(place_name)

# Define starting and ending points (latitude, longitude)
start_point = (-6.2209825, 106.6666885)  # Monas
end_point = (-6.17539, 106.82715)  # Kota Tua

# Get the nearest nodes to the starting and ending points
start_node = ox.distance.nearest_nodes(G, start_point[1], start_point[0])
end_node = ox.distance.nearest_nodes(G, end_point[1], end_point[0])

# Function to implement Dijkstra's algorithm and log the process
def dijkstra_search(G, start_node, end_node):
    pq = PriorityQueue()
    pq.put((0, start_node))
    visited = {start_node: 0}
    path = {start_node: None}

    while not pq.empty():
        (cost, current_node) = pq.get()
        if current_node == end_node:
            break

        for neighbor in G.neighbors(current_node):
            new_cost = cost + G.edges[current_node, neighbor, 0]['length']
            if neighbor not in visited or new_cost < visited[neighbor]:
                visited[neighbor] = new_cost
                pq.put((new_cost, neighbor))
                path[neighbor] = current_node
                print(f"Visiting node {neighbor} with cost {new_cost}")

    # Reconstruct path
    shortest_path = []
    step = end_node
    while step is not None:
        shortest_path.insert(0, step)
        step = path[step]

    return shortest_path

# Compute the shortest path using Dijkstra's algorithm
shortest_path = dijkstra_search(G, start_node, end_node)

# Function to animate the search process
def animate_search(path, G, ax):
    lines = []
    nodes = list(path)

    def update(num):
        if num < len(nodes) - 1:
            u, v = nodes[num], nodes[num + 1]
            x = [G.nodes[u]['x'], G.nodes[v]['x']]
            y = [G.nodes[u]['y'], G.nodes[v]['y']]
            line, = ax.plot(x, y, color='blue')
            lines.append(line)
            print(f"Step {num + 1}: from node {u} to node {v}")
        return lines

    return update

# Create a figure and axis
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_title("Shortest Path Search in Jakarta")

# Plot the network
ox.plot_graph(G, ax=ax, node_size=0, edge_color='gray', show=False, close=False)

# Add start and end points to the plot
ax.plot(G.nodes[start_node]['x'], G.nodes[start_node]['y'], 'go', markersize=10)
ax.plot(G.nodes[end_node]['x'], G.nodes[end_node]['y'], 'ro', markersize=10)

# Create animation
ani = animation.FuncAnimation(fig, animate_search(shortest_path, G, ax), frames=len(shortest_path), interval=200, blit=False)

# Save the final path as a PNG image
if shortest_path:
    fig.savefig("shortest_path.png")

# Show the animation
plt.show()
