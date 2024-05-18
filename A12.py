import heapq
import numpy as np
from scipy.sparse.csgraph import dijkstra as dijkstra_sparse
from scipy.sparse import csr_matrix
import math

# Función para leer el grafo desde el archivo "grafo.txt"
def leer_grafo(nombre_archivo):
    grafo = []
    with open(nombre_archivo, 'r') as archivo:
        for linea in archivo:
            fila = list(map(int, linea.split()))
            grafo.append(fila)
    return grafo

# Función para leer el archivo "archivo.txt" y obtener la matriz de adyacencia de la ruta
def obtener_matriz_ruta(nombre_archivo, num_ciudades):
    ruta = np.zeros((num_ciudades, num_ciudades))
    with open(nombre_archivo, 'r') as archivo:
        for linea in archivo:
            ciudades = linea.strip().split()
            if len(ciudades) != 2:
                print(f"Advertencia: Ignorando línea incorrecta en {nombre_archivo}: {linea.strip()}")
                continue
            ciudad_actual, ciudad_siguiente = ciudades
            ciudad_actual = ord(ciudad_actual) - ord('A')
            ciudad_siguiente = ord(ciudad_siguiente) - ord('A')
            ruta[ciudad_actual, ciudad_siguiente] = 1
            ruta[ciudad_siguiente, ciudad_actual] = 1
    return ruta

# Función para leer la matriz de capacidades desde el archivo "capacidades.txt"
def leer_capacidades(nombre_archivo):
    capacidades = []
    with open(nombre_archivo, 'r') as archivo:
        for linea in archivo:
            fila = list(map(int, linea.split()))
            capacidades.append(fila)
    return capacidades

# Implementación del algoritmo de Dijkstra
def dijkstra(grafo, nodo_inicial):
    n = len(grafo)
    if nodo_inicial < 0 or nodo_inicial >= n:
        print("Error: El nodo inicial está fuera del rango válido.")
        return None
    
    distancia = [float('inf')] * n
    distancia[nodo_inicial] = 0
    visitado = [False] * n
    cola_prioridad = [(0, nodo_inicial)]

    while cola_prioridad:
        dist_actual, nodo_actual = heapq.heappop(cola_prioridad)
        if visitado[nodo_actual]:
            continue
        visitado[nodo_actual] = True

        for vecino, peso in enumerate(grafo[nodo_actual]):
            if peso > 0 and not visitado[vecino]:
                nueva_distancia = dist_actual + peso
                if nueva_distancia < distancia[vecino]:
                    distancia[vecino] = nueva_distancia
                    heapq.heappush(cola_prioridad, (nueva_distancia, vecino))

    return distancia

# Implementación del algoritmo de Prim para encontrar el MST
def prim_mst(grafo):
    num_nodos = len(grafo)
    seleccionados = [False] * num_nodos
    distancias = [float('inf')] * num_nodos
    padres = [-1] * num_nodos
    distancias[0] = 0

    for _ in range(num_nodos):
        min_dist = float('inf')
        u = -1

        for i in range(num_nodos):
            if not seleccionados[i] and distancias[i] < min_dist:
                min_dist = distancias[i]
                u = i

        seleccionados[u] = True

        for v in range(num_nodos):
            if grafo[u][v] > 0 and not seleccionados[v] and grafo[u][v] < distancias[v]:
                distancias[v] = grafo[u][v]
                padres[v] = u

    return padres

def imprimir_mst(padres, grafo):
    print("Arista \tPeso")
    for i in range(1, len(padres)):
        print(f"{padres[i]} - {i} \t{grafo[i][padres[i]]}")

# Función para encontrar la ruta más corta que visita cada colonia exactamente una vez y regresa al origen
def encontrar_ruta_corta(matriz_ruta, nodo_inicial):
    num_ciudades = matriz_ruta.shape[0]
    if nodo_inicial < 0 or nodo_inicial >= num_ciudades:
        print("Error: El nodo inicial está fuera del rango válido.")
        return None
    
    visitado = [False] * num_ciudades
    ruta = [nodo_inicial]

    def backtrack(ciudad_actual):
        if len(ruta) == num_ciudades:
            return matriz_ruta[ciudad_actual, nodo_inicial]
        for ciudad_siguiente in range(num_ciudades):
            if matriz_ruta[ciudad_actual, ciudad_siguiente] and not visitado[ciudad_siguiente]:
                visitado[ciudad_siguiente] = True
                ruta.append(ciudad_siguiente)
                if backtrack(ciudad_siguiente):
                    return True
                ruta.pop()
                visitado[ciudad_siguiente] = False
        return False

    visitado[nodo_inicial] = True
    if backtrack(nodo_inicial):
        return ruta
    else:
        return None

# Función para imprimir la ruta más corta
def imprimir_ruta(ruta):
    if ruta:
        ruta_nombres = [chr(ciudad + ord('A')) for ciudad in ruta]
        print("Ruta más corta que visita cada colonia exactamente una vez y regresa al origen:")
        print(" -> ".join(ruta_nombres))
    else:
        print("No se encontró ninguna ruta válida.")

# Función para encontrar el flujo máximo de información
def flujo_maximo(capacidades, nodo_inicial, nodo_final):
    matriz_capacidades = csr_matrix(capacidades)
    flujo = dijkstra_sparse(matriz_capacidades, directed=False, indices=nodo_inicial, return_predecessors=False)
    return flujo

# Función para leer las ubicaciones de las centrales y la nueva contratación
def leer_ubicaciones(nombre_archivo):
    ubicaciones = []
    with open(nombre_archivo, 'r') as archivo:
        for linea in archivo:
            x, y = map(float, linea.strip().split())
            ubicaciones.append((x, y))
    return ubicaciones

# Función para encontrar la central más cercana a una nueva contratación
def central_mas_cercana(centrales, nueva_central):
    min_dist = float('inf')
    central_mas_cercana = None
    for central in centrales:
        dist = math.sqrt((central[0] - nueva_central[0]) ** 2 + (central[1] - nueva_central[1]) ** 2)
        if dist < min_dist:
            min_dist = dist
            central_mas_cercana = central
    return central_mas_cercana, min_dist

# Llamada a las funciones
grafo = leer_grafo("grafo.txt")
nodo_inicial = 0  # Puedes elegir cualquier nodo como inicial
distancias = dijkstra(grafo, nodo_inicial)
print("Distancias desde el nodo inicial:")
print(distancias)

# Encontrar y mostrar el MST
padres = prim_mst(grafo)
imprimir_mst(padres, grafo)

num_ciudades = len(grafo)
matriz_ruta = obtener_matriz_ruta("archivo.txt", num_ciudades)
ruta = encontrar_ruta_corta(matriz_ruta, nodo_inicial)
imprimir_ruta(ruta)

capacidades = leer_capacidades("capacidades.txt")
nodo_final = 0  # Puedes elegir cualquier nodo como final
flujo = flujo_maximo(capacidades, nodo_inicial, nodo_final)
print(f"Flujo máximo de información: {flujo}")

# Leer ubicaciones de las centrales y la nueva contratación
ubicaciones = leer_ubicaciones("ubicaciones.txt")
centrales = ubicaciones[:-1]
nueva_central = ubicaciones[-1]

# Encontrar la central más cercana a la nueva contratación
central_cercana, distancia = central_mas_cercana(centrales, nueva_central)
print(f"La central más cercana a la nueva contratación está en {central_cercana} con una distancia de {distancia:.2f} kilómetros.")
