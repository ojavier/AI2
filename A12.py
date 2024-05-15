import os
from collections import deque

class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for _ in range(vertices)] for _ in range(vertices)]

    def min_key(self, key, mst_set):
        min_val = float('inf')
        min_index = None

        for v in range(self.V):
            if key[v] < min_val and not mst_set[v]:
                min_val = key[v]
                min_index = v

        return min_index

    def nearest_neighbor(self):
        path = []
        mst_set = [False] * self.V
        key = [float('inf')] * self.V

        current = 0
        path.append(current)
        mst_set[current] = True

        for _ in range(self.V - 1):
            for v in range(self.V):
                if (self.graph[current][v] > 0
                        and not mst_set[v]
                        and self.graph[current][v] < key[v]):
                    key[v] = self.graph[current][v]
            next_node = self.min_key(key, mst_set)
            path.append(next_node)
            mst_set[next_node] = True
            current = next_node

        path.append(0)  # Regresa al nodo de origen
        return path

    def print_tsp_path(self, path):
        print("Ruta más corta:")
        for i in range(len(path)):
            print(chr(ord('A') + path[i]), end=" -> " if i < len(path) - 1 else "\n")
    
    def bfs(self, s, t, parent):
        visited = [False] * self.V
        queue = deque()
        queue.append(s)
        visited[s] = True
        
        while queue:
            u = queue.popleft()
            for v in range(self.V):
                if not visited[v] and self.graph[u][v] > 0:
                    queue.append(v)
                    visited[v] = True
                    parent[v] = u
        return visited[t]

    def ford_fulkerson(self, source, sink):
        parent = [-1] * self.V
        max_flow = 0
        
        while self.bfs(source, sink, parent):
            path_flow = float('inf')
            s = sink
            while s != source:
                path_flow = min(path_flow, self.graph[parent[s]][s])
                s = parent[s]
            max_flow += path_flow
            v = sink
            while v != source:
                u = parent[v]
                self.graph[u][v] -= path_flow
                self.graph[v][u] += path_flow
                v = parent[v]
        return max_flow


def read_adjacency_matrix(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
        n = len(lines)
        graph = Graph(n)

        for i in range(n):
            line = lines[i].strip().split()
            for j in range(n):
                graph.graph[i][j] = int(line[j])

        return graph


if __name__ == "__main__":
    current_directory = os.path.dirname(__file__)
    filename = os.path.join(current_directory, "archivo.txt")

    graph = read_adjacency_matrix(filename)

    path = graph.nearest_neighbor()

    graph.print_tsp_path(path)

    # Leer la matriz de capacidades máximas
    capacity_filename = os.path.join(current_directory, "capacidades.txt")
    capacities_graph = read_adjacency_matrix(capacity_filename)

    # Calcular y mostrar el flujo máximo de información
    source = 0
    sink = graph.V - 1
    max_flow = capacities_graph.ford_fulkerson(source, sink)
    print("Flujo máximo de información:", max_flow)
