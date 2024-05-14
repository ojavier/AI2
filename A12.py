import os


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
