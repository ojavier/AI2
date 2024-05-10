import sys


# representación del grafo
class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for _ in range(vertices)] for _ in range(vertices)]

    # vértice con la clave mínima
    def min_key(self, key, mst_set):
        min_val = sys.maxsize
        min_index = None

        for v in range(self.V):
            if key[v] < min_val and not mst_set[v]:
                min_val = key[v]
                min_index = v

        return min_index

    # algoritmo de Prim para encontrar el árbol de expansión mínima
    def prim_mst(self):
        parent = [None] * self.V
        key = [sys.maxsize] * self.V
        key[0] = 0
        mst_set = [False] * self.V

        parent[0] = -1

        for _ in range(self.V):
            u = self.min_key(key, mst_set)
            mst_set[u] = True

            for v in range(self.V):
                if self.graph[u][v] > 0 and not mst_set[v] and self.graph[u][v] < key[v]:
                    parent[v] = u
                    key[v] = self.graph[u][v]

        return parent
    
    # aristas del árbol de expansión mínima
    def print_mst(self, parent):
        print("Edge \tWeight")
        for i in range(1, self.V):
            print(parent[i], "-", i, "\t", self.graph[i][parent[i]])

# Función para leer la matriz de adyacencia desde un archivo
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

# Main
if __name__ == "__main__":
    # Lee el nombre del archivo como argumento de línea de comandos
    if len(sys.argv) != 2:
        print("Uso: python programa.py archivo_entrada")
        sys.exit()

    filename = sys.argv[1]

    # Lee la matriz de adyacencia desde el archivo
    graph = read_adjacency_matrix(filename)

    # Encuentra el árbol de expansión mínima utilizando el algoritmo de Prim
    parent = graph.prim_mst()

    # Imprime el árbol de expansión mínima
    graph.print_mst(parent)
