import queue
import networkx as nx
import matplotlib.pyplot as plt
import time

def order_bfs(graph, start_node):
    visited = set() # To know which nodes we have already visited
    q = queue.Queue() # FIFO: First In First Out
    q.put(start_node) 
    order = [] # The nodes in the correct order

    while not q.empty():
        vertex = q.get() # Our vertex will be whatever pops out of 
        # the queue 
        if vertex not in visited:
            order.append(vertex)
            visited.add(vertex) # To know we already visited it
            for node in graph[vertex]: # What we do here is that 
                # for each vertex, we visit the neighboring nodes in
                # the graph, and add them to the queue by the 
                # command 'q.put(node)' to be processed in the next
                # iteration of the loop. 
                if node not in visited:
                    q.put(node)
    return order

def order_dfs(graph, start_node, visited = None):
    if visited is None:
        visited = set()

    # In this algorithm we use recursion because we have a repeated
    # task that can be done on progressively smaller parts of the 
    # problem until it reaches an "end" condition. 
    order = []

    if start_node not in visited:
        order.append(start_node)
        visited.add(start_node)
        for node in graph[start_node]:
            if node not in visited:
                order.extend(order_dfs(graph, node, visited))

    # Basically we start with the whole graph, we move, and create 
    # a new subgraph that doesn't contain the node we started with, 
    # repeating this logic over and over again, until it has moved
    # through the whole "path" of a branch of the node. Once it has 
    # done that, it goes "up" in the sense that it comes back to the
    # last intersection there was, and explores another path, 
    # effectively "bubbling up" the 'order' variable. 
    return order

def visualize_search(order, title, G, pos):
    plt.figure()
    plt.title(title)
    for i, node in enumerate(order, start=1):
        plt.clf()
        # 'clf' stands for "clear figure". 
        plt.title(title)
        nx.draw(G, pos, with_labels = True, node_color=['red' if n == node else 'green' for n in G.nodes])
        # This creates a sense of animation in which the current node is the red one while all the others
        # remain green. 
        plt.draw()
        plt.pause(1.5)
    plt.show()
    time.sleep(0.5)

def generate_connected_random_graph(n, m):
    while True:
        G = nx.gnm_random_graph(n,m)
        if nx.is_connected(G):
            return G


# --- Examples ---

#G = nx.Graph()
G = generate_connected_random_graph(20,30)
#G.add_edges_from([('A', 'B'), ('A', 'C'), ('B', 'D'), ('B','E'), ('C','F'), ('C','G')])
pos = nx.spring_layout(G)

#visualize_search(order_bfs(G, start_node='A'), title="BFS Visualization", G=G, pos=pos)
#visualize_search(order_dfs(G, start_node='A'), title="DFS Visualization", G=G, pos=pos)

#visualize_search(order_bfs(G, start_node=0), title="BFS Visualization", G=G, pos=pos)
visualize_search(order_dfs(G, start_node=0), title="DFS Visualization", G=G, pos=pos)
