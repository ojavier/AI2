import heapq
import numpy as np
from scipy.sparse.csgraph import dijkstra as dijkstra_sparse
from scipy.sparse import csr_matrix
import math

# Función para leer el archivo de entrada con distancias, capacidades y ubicaciones
def leer_datos_completos(nombre_archivo):
    try:
        with open(nombre_archivo, 'r') as archivo:
            # Leer el número de colonias
            while True:
                primera_linea = archivo.readline().strip()
                if primera_linea and primera_linea[0].isdigit():
                    break
            n = int(primera_linea.split()[0])

            # Leer la matriz de distancias
            distancias = []
            for _ in range(n):
                while True:
                    linea = archivo.readline().strip()
                    if linea and linea[0].isdigit():
                        break
                fila = list(map(int, linea.split()))
                distancias.append(fila)
            archivo.readline()  # Leer la línea en blanco que separa las matrices

            # Leer la matriz de capacidades
            capacidades = []
            for _ in range(n):
                while True:
                    linea = archivo.readline().strip()
                    if linea and linea[0].isdigit():
                        break
                fila = list(map(int, linea.split()))
                capacidades.append(fila)
            archivo.readline()  # Leer la línea en blanco que separa las matrices de las ubicaciones

            # Leer las ubicaciones (n centrales + 1 nueva central)
            ubicaciones = []
            for _ in range(n + 1):
                while True:
                    linea = archivo.readline().strip()
                    if linea and linea[0] == '(':
                        break
                x, y = map(float, linea.strip('()').split(','))
                ubicaciones.append((x, y))

        return distancias, capacidades, ubicaciones
    except FileNotFoundError:
        print(f"Error: El archivo '{nombre_archivo}' no se encontró.")
    except ValueError as ve:
        print(f"Error de valor: {ve}")
    except Exception as e:
        print(f"Se produjo un error inesperado: {e}")
        return None

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

# Función para leer el grafo desde el archivo "grafo.txt"
def leer_grafo(nombre_archivo):
    grafo = []
    with open(nombre_archivo, 'r') as archivo:
        for linea in archivo:
            fila = list(map(int, linea.split()))
            grafo.append(fila)
    return grafo

# Implementación del algoritmo de Nearest Neighbor para encontrar la ruta más corta
def encontrar_ruta_corta(matriz_ruta, nodo_inicial):
    num_ciudades = matriz_ruta.shape[0]
    if nodo_inicial < 0 or nodo_inicial >= num_ciudades:
        print("Error: El nodo inicial está fuera del rango válido.")
        return None
    
    visitado = [False] * num_ciudades
    ruta = [nodo_inicial]
    visitado[nodo_inicial] = True
    ciudad_actual = nodo_inicial

    while len(ruta) < num_ciudades:
        min_dist = float('inf')
        ciudad_siguiente = None
        for ciudad in range(num_ciudades):
            if not visitado[ciudad] and matriz_ruta[ciudad_actual, ciudad] > 0 and matriz_ruta[ciudad_actual, ciudad] < min_dist:
                min_dist = matriz_ruta[ciudad_actual, ciudad]
                ciudad_siguiente = ciudad
        if ciudad_siguiente is not None:
            ruta.append(ciudad_siguiente)
            visitado[ciudad_siguiente] = True
            ciudad_actual = ciudad_siguiente
        else:
            print("Error: No se encontró una ciudad siguiente válida.")
            return None
    
    # Volver al nodo inicial para completar el ciclo
    ruta.append(nodo_inicial)
    return ruta

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
def main():
    nombre_archivo = 'ubicaciones.txt'

    # Leer todas las entradas desde el archivo
    resultado = leer_datos_completos(nombre_archivo)
    if resultado is None:
        print("Error al leer los datos. Saliendo del programa.")
        return

    distancias, capacidades, ubicaciones = resultado

    # Separar las ubicaciones en centrales y la nueva central
    centrales = ubicaciones[:-1]
    nueva_central = ubicaciones[-1]

    # Convertir las matrices de distancias y capacidades a listas para su uso
    grafo = distancias

    nodo_inicial = 0  # Puedes elegir cualquier nodo como inicial

    # Calcular las distancias usando Dijkstra
    distancias_dijkstra = dijkstra(grafo, nodo_inicial)
    print("Distancias desde el nodo inicial:")
    print(distancias_dijkstra)

    # Encontrar y mostrar el MST
    padres = prim_mst(grafo)
    imprimir_mst(padres, grafo)

    num_ciudades = len(grafo)
    matriz_ruta = np.array(grafo)
    ruta = encontrar_ruta_corta(matriz_ruta, nodo_inicial)
    imprimir_ruta(ruta)

    nodo_final = 0  # Puedes elegir cualquier nodo como final
    flujo = flujo_maximo(capacidades, nodo_inicial, nodo_final)
    print(f"Flujo máximo de información: {flujo}")

    # Encontrar la central más cercana a la nueva contratación
    central_cercana, distancia = central_mas_cercana(centrales, nueva_central)
    print(f"La central más cercana a la nueva contratación está en {central_cercana} con una distancia de {distancia:.2f} kilómetros.")

if __name__ == "__main__":
    main()
