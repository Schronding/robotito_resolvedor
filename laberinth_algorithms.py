import collections
import networkx as nx
import matplotlib.pyplot as plt
import traceback # Para imprimir errores detallados

# --- Definiciones del Laberinto ---
WALL_CHAR = '#'
PATH_CHAR = ' '
START_CHAR = 'S'
END_CHAR = 'E'

# --- Para la conversión a números ---
WALL = 1
PATH = 0
START = 2
END = 3

# --- Direcciones y Movimientos ---
DIRECTIONS_map = {
    'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)
}
NEIGHBOR_ORDER_global = ['E', 'S', 'W', 'N'] # Preferencia para DFS estándar

# Mapeos para algoritmos basados en dirección indexada (0:N, 1:E, 2:S, 3:W)
DIR_TO_IDX = {(-1,0):0, (0,1):1, (1,0):2, (0,-1):3} 
IDX_TO_DR_DC = {0:(-1,0), 1:(0,1), 2:(1,0), 3:(0,-1)}

def parse_laberinto(laberinto_str_list):
    """Convierte el laberinto de strings a una representación numérica y encuentra S y E."""
    mapa_numerico = []
    pos_inicio = None
    pos_fin = None
    height = len(laberinto_str_list)
    width = 0
    if height > 0:
        width = len(laberinto_str_list[0])

    for r_idx, fila_str in enumerate(laberinto_str_list):
        if len(fila_str) != width:
            raise ValueError(f"Todas las filas deben tener la misma longitud. Fila {r_idx} tiene {len(fila_str)}, se esperaba {width}")
        fila_num = []
        for c_idx, char in enumerate(fila_str):
            if char == WALL_CHAR:
                fila_num.append(WALL)
            elif char == PATH_CHAR:
                fila_num.append(PATH)
            elif char == START_CHAR:
                fila_num.append(START)
                if pos_inicio:
                    raise ValueError("Múltiples puntos de inicio 'S' encontrados.")
                pos_inicio = (r_idx, c_idx)
            elif char == END_CHAR:
                fila_num.append(END)
                if pos_fin:
                    raise ValueError("Múltiples puntos de fin 'E' encontrados.")
                pos_fin = (r_idx, c_idx)
            else:
                # Tratar caracteres no reconocidos como muros
                print(f"Advertencia: Caracter '{char}' no reconocido en ({r_idx},{c_idx}). Tratado como muro.")
                fila_num.append(WALL)
        mapa_numerico.append(fila_num)

    if not pos_inicio:
        raise ValueError("No se encontró el punto de inicio 'S' en el laberinto.")
    if not pos_fin:
        raise ValueError("No se encontró el punto de fin 'E' en el laberinto.")

    return mapa_numerico, pos_inicio, pos_fin, height, width

def encontrar_camino_seguidor_pared(laberinto_num, inicio, fin, height, width, tipo_seguidor):
    """
    Encuentra un camino usando el algoritmo de seguidor de pared (izquierda o derecha).
    'tipo_seguidor' puede ser 'izquierda' o 'derecha'.
    """
    camino_actual = [inicio]
    r, c = inicio
    
    dir_mirada_idx = -1 # Índice de la dirección actual de "mirada" del robot

    # El robot debe dar un primer paso para establecer una dirección de mirada.
    # Intenta moverse en el orden de NEIGHBOR_ORDER_global.
    for dr_char_init in NEIGHBOR_ORDER_global: 
        dr_init, dc_init = DIRECTIONS_map[dr_char_init]
        nr_init, nc_init = r + dr_init, c + dc_init
        
        if 0 <= nr_init < height and 0 <= nc_init < width and laberinto_num[nr_init][nc_init] != WALL:
            # Movimiento inicial válido
            dir_mirada_idx = DIR_TO_IDX[(dr_init, dc_init)] # Establecer dirección de mirada
            r, c = nr_init, nc_init # Moverse a la nueva celda
            camino_actual.append((r,c))
            break 
    
    if dir_mirada_idx == -1: # Si el inicio está completamente bloqueado
        return None

    max_pasos_simulacion = height * width * 2 # Salvaguarda contra bucles infinitos
    pasos_realizados = 0 

    while (r, c) != fin and pasos_realizados < max_pasos_simulacion:
        pasos_realizados += 1
        
        # Determinar el orden de prueba de las direcciones relativas
        if tipo_seguidor == 'izquierda': # Mantener la pared a la IZQUIERDA
            # Orden de prueba: Izquierda Relativa, Recto, Derecha Relativa, Atrás
            idx_orden_prueba = [
                (dir_mirada_idx - 1 + 4) % 4, # Izquierda
                dir_mirada_idx,               # Recto
                (dir_mirada_idx + 1) % 4,     # Derecha
                (dir_mirada_idx + 2) % 4      # Atrás (180 grados)
            ]
        else: # 'derecha', mantener la pared a la DERECHA
            # Orden de prueba: Derecha Relativa, Recto, Izquierda Relativa, Atrás
            idx_orden_prueba = [
                (dir_mirada_idx + 1) % 4,     # Derecha
                dir_mirada_idx,               # Recto
                (dir_mirada_idx - 1 + 4) % 4, # Izquierda
                (dir_mirada_idx + 2) % 4      # Atrás
            ]

        movimiento_efectuado_este_paso = False
        for idx_nueva_direccion_mirada in idx_orden_prueba:
            dr_new, dc_new = IDX_TO_DR_DC[idx_nueva_direccion_mirada]
            nr_new, nc_new = r + dr_new, c + dc_new # Posición tentativa

            # Comprobar si el movimiento es válido (dentro del laberinto y no es pared)
            if 0 <= nr_new < height and 0 <= nc_new < width and laberinto_num[nr_new][nc_new] != WALL:
                r, c = nr_new, nc_new # Moverse
                camino_actual.append((r,c))
                dir_mirada_idx = idx_nueva_direccion_mirada # Actualizar dirección de mirada
                movimiento_efectuado_este_paso = True
                break # Movimiento realizado, salir del bucle de prueba de direcciones
        
        if not movimiento_efectuado_este_paso:
            # Atascado (no debería ocurrir en laberintos simples conexos con la opción "Atrás")
            # print(f"Seguidor de pared ({tipo_seguidor}) se atascó en {r,c} mirando {dir_mirada_idx}.")
            return None 

    if (r,c) == fin:
        return camino_actual
    else: # Límite de pasos alcanzado o atascado sin llegar al fin
        # print(f"Seguidor de pared ({tipo_seguidor}) no encontró el fin. Última pos: {r,c}, Pasos: {pasos_realizados}")
        return None

def encontrar_N_caminos_dfs(laberinto_num, inicio, fin, height, width, N_caminos_max, caminos_existentes_coords_set):
    """Encuentra hasta N caminos únicos usando DFS, evitando los ya presentes en caminos_existentes_coords_set."""
    caminos_encontrados_dfs = []
    
    laberinto_transitable = [list(fila) for fila in laberinto_num]
    if laberinto_transitable[inicio[0]][inicio[1]] == START:
        laberinto_transitable[inicio[0]][inicio[1]] = PATH
    if laberinto_transitable[fin[0]][fin[1]] == END:
         laberinto_transitable[fin[0]][fin[1]] = PATH

    pila = collections.deque([(inicio, [inicio])]) 

    while pila:
        (r_curr, c_curr), camino_parcial = pila.pop()

        if (r_curr, c_curr) == fin:
            camino_tuple = tuple(camino_parcial)
            if camino_tuple not in caminos_existentes_coords_set: # Solo si es un camino nuevo
                caminos_encontrados_dfs.append(list(camino_parcial))
                if len(caminos_encontrados_dfs) >= N_caminos_max:
                    return caminos_encontrados_dfs # Se alcanzó el número deseado de nuevos caminos
            continue 

        for dr_char in NEIGHBOR_ORDER_global: # Usar orden global para exploración DFS
            dr, dc = DIRECTIONS_map[dr_char]
            nr_next, nc_next = r_curr + dr, c_curr + dc

            if 0 <= nr_next < height and 0 <= nc_next < width and \
               laberinto_transitable[nr_next][nc_next] != WALL and \
               (nr_next, nc_next) not in camino_parcial: 
                
                nuevo_camino_parcial = list(camino_parcial)
                nuevo_camino_parcial.append((nr_next, nc_next))
                pila.append(((nr_next, nc_next), nuevo_camino_parcial))
    
    return caminos_encontrados_dfs # Puede retornar menos de N_caminos_max si no se encuentran más

def encontrar_N_caminos_bfs(laberinto_num, inicio, fin, height, width, N_caminos_max, caminos_existentes_coords_set):
    """
    Encuentra hasta N caminos únicos usando una estrategia tipo BFS.
    Los caminos se encuentran explorando en anchura y se evita revisitar nodos dentro del mismo camino parcial.
    """
    caminos_encontrados_bfs = []
    
    # Copia del laberinto para que S y E sean transitables internamente
    laberinto_transitable = [list(fila) for fila in laberinto_num]
    if laberinto_transitable[inicio[0]][inicio[1]] == START:
        laberinto_transitable[inicio[0]][inicio[1]] = PATH
    if laberinto_transitable[fin[0]][fin[1]] == END:
         laberinto_transitable[fin[0]][fin[1]] = PATH

    # Cola para BFS: (posición_actual, camino_hasta_ahora)
    queue = collections.deque([(inicio, [inicio])])
    
    # No se usa un conjunto global de 'visitados' para la búsqueda BFS de múltiples caminos,
    # ya que un nodo podría ser parte de diferentes caminos. 
    # La condición `(nr_next, nc_next) not in camino_parcial` previene ciclos dentro de un mismo camino.

    while queue:
        (r_curr, c_curr), camino_parcial = queue.popleft() # FIFO

        # Explorar vecinos (usar NEIGHBOR_ORDER_global para mantener consistencia o variarlo para diversidad)
        for dr_char in NEIGHBOR_ORDER_global:
            dr, dc = DIRECTIONS_map[dr_char]
            nr_next, nc_next = r_curr + dr, c_curr + dc

            if 0 <= nr_next < height and 0 <= nc_next < width and \
               laberinto_transitable[nr_next][nc_next] != WALL and \
               (nr_next, nc_next) not in camino_parcial: # Evitar ciclos en el camino actual
                
                nuevo_camino_parcial = list(camino_parcial)
                nuevo_camino_parcial.append((nr_next, nc_next))

                if (nr_next, nc_next) == fin: # Se llegó al destino
                    camino_tuple = tuple(nuevo_camino_parcial)
                    if camino_tuple not in caminos_existentes_coords_set:
                        caminos_encontrados_bfs.append(nuevo_camino_parcial)
                        if len(caminos_encontrados_bfs) >= N_caminos_max:
                            return caminos_encontrados_bfs # Se alcanzó el límite
                else: # Continuar la búsqueda desde este nuevo punto
                    queue.append(((nr_next, nc_next), nuevo_camino_parcial))
    
    return caminos_encontrados_bfs # Retorna los caminos encontrados (podrían ser menos de N_caminos_max)

def convertir_camino_a_instrucciones(camino_coordenadas):
    if not camino_coordenadas or len(camino_coordenadas) < 2:
        return ""
    instrucciones = []
    dr_actual, dc_actual = (camino_coordenadas[1][0] - camino_coordenadas[0][0],
                            camino_coordenadas[1][1] - camino_coordenadas[0][1])
    if (dr_actual, dc_actual) not in DIR_TO_IDX:
        raise ValueError(f"Movimiento inicial no válido: {camino_coordenadas[0]}->{camino_coordenadas[1]}")
    dir_idx_actual = DIR_TO_IDX[(dr_actual, dc_actual)]
    instrucciones.append('F')

    for i in range(1, len(camino_coordenadas) - 1):
        dr_nuevo, dc_nuevo = (camino_coordenadas[i+1][0] - camino_coordenadas[i][0],
                              camino_coordenadas[i+1][1] - camino_coordenadas[i][1])
        if (dr_nuevo, dc_nuevo) not in DIR_TO_IDX:
            raise ValueError(f"Movimiento no válido en camino: {camino_coordenadas[i]}->{camino_coordenadas[i+1]}")
        dir_idx_nuevo = DIR_TO_IDX[(dr_nuevo, dc_nuevo)]
        
        if dir_idx_nuevo == dir_idx_actual:
            instrucciones.append('F')
        elif dir_idx_nuevo == (dir_idx_actual + 1) % 4: # Derecha
            instrucciones.append('R'); instrucciones.append('F')
        elif dir_idx_nuevo == (dir_idx_actual - 1 + 4) % 4: # Izquierda
            instrucciones.append('L'); instrucciones.append('F')
        else: # Media vuelta
            instrucciones.append('R'); instrucciones.append('R'); instrucciones.append('F')
        dir_idx_actual = dir_idx_nuevo
    return "".join(instrucciones)

def visualizar_camino_en_laberinto(laberinto_str_list, camino_coordenadas):
    if not camino_coordenadas: return ["Laberinto original"] + laberinto_str_list
    lab_visual = [list(fila) for fila in laberinto_str_list]
    for r_coord, c_coord in camino_coordenadas:
        if lab_visual[r_coord][c_coord] != START_CHAR and lab_visual[r_coord][c_coord] != END_CHAR:
            lab_visual[r_coord][c_coord] = '*'
    # Asegurar que S y E se muestren con su caracter original
    lab_visual[camino_coordenadas[0][0]][camino_coordenadas[0][1]] = START_CHAR
    lab_visual[camino_coordenadas[-1][0]][camino_coordenadas[-1][1]] = END_CHAR
    return ["Laberinto con camino marcado (*):"] + ["".join(fila) for fila in lab_visual]

def camino_a_grafo_ponderado(camino_coordenadas):
    if not camino_coordenadas or len(camino_coordenadas) < 2:
        return nx.Graph(), {}, {}
    G = nx.Graph(); pos_layout = {}; edge_labels = {}
    nodo_grafo_actual = camino_coordenadas[0]
    G.add_node(nodo_grafo_actual)
    pos_layout[nodo_grafo_actual] = (camino_coordenadas[0][1], -camino_coordenadas[0][0])
    pasos_segmento = 0
    dr_seg, dc_seg = (camino_coordenadas[1][0] - camino_coordenadas[0][0],
                      camino_coordenadas[1][1] - camino_coordenadas[0][1])
    dir_actual_segmento = (dr_seg, dc_seg)

    for i in range(len(camino_coordenadas) - 1):
        pasos_segmento += 1
        coord_sig = camino_coordenadas[i+1]
        es_final = (i + 1 == len(camino_coordenadas) - 1)
        cambio_dir = False
        if not es_final:
            dr_prox, dc_prox = (camino_coordenadas[i+2][0] - coord_sig[0],
                                camino_coordenadas[i+2][1] - coord_sig[1])
            if (dr_prox, dc_prox) != dir_actual_segmento: cambio_dir = True
        
        if es_final or cambio_dir:
            nuevo_nodo = coord_sig
            G.add_node(nuevo_nodo)
            pos_layout[nuevo_nodo] = (nuevo_nodo[1], -nuevo_nodo[0])
            G.add_edge(nodo_grafo_actual, nuevo_nodo, weight=pasos_segmento)
            edge_labels[(nodo_grafo_actual, nuevo_nodo)] = pasos_segmento
            nodo_grafo_actual = nuevo_nodo
            pasos_segmento = 0
            if not es_final: dir_actual_segmento = (dr_prox, dc_prox)
    return G, pos_layout, edge_labels

def visualizar_grafo_de_camino(G, pos, edge_labels, title):
    if not G.nodes():
        print(f"Grafo para '{title}' vacío.")
        return
    plt.figure(num = title, figsize=(9, 7)) # Ligeramente más pequeño
    node_labels = {node: f"({node[0]},{node[1]})" for node in G.nodes()}
    nx.draw(G, pos, with_labels=True, labels=node_labels, node_color='lightgreen', 
            node_size=1000, font_size=7, font_weight='normal')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='darkred', font_size=7)
    plt.title(title, fontsize=10)
    plt.draw()
    plt.pause(0.01) # Pequeña pausa para ayudar al renderizado

# --- Representacion grafica de nuestro laberinto real ---
laberinto_real = [
        "############",
        "S    #     #",
        "####     ###",
        "#          #",
        "# # ####   #",
        "# #   #    #",
        "# #   #  ###",
        "#   #      #",
        "# #######  #",
        "#          #",
        "##########E#"] 

# --- Procesamiento ---
try:
    print("Parseando laberinto...")
    laberinto_num, pos_inicio, pos_fin, alto, ancho = parse_laberinto(laberinto_real)
    print(f"Laberinto parseado. Inicio: {pos_inicio}, Fin: {pos_fin}, Dimensiones: {alto}x{ancho}")
    
    caminos_finales_para_mostrar = []
    coords_caminos_vistos = set() # Al añadir el set nos aseguramos que cada camino que sacamos es unico, por lo que, aunque los pesos puedan
    # ser los mismos, cada grafo es distinto y puede ser comparado como tal
    
    MAX_CAMINOS_A_MOSTRAR = 6 # Esta variable me ayuda para no crear un cantidad tan grande de caminos diferentes como para alentar mi computadora
    # deberia ser capaz de ser modificada libremente

    # 1. Camino Seguidor de Pared Izquierda
    print("\nBuscando camino 'Seguidor de Pared Izquierda'...")
    cam_izq = encontrar_camino_seguidor_pared(laberinto_num, pos_inicio, pos_fin, alto, ancho, 'izquierda')
    if cam_izq:
        print(f"Camino 'Izquierda' encontrado (longitud {len(cam_izq)}).")
        caminos_finales_para_mostrar.append({"camino": cam_izq, "nombre": "Pared Izquierda"})
        coords_caminos_vistos.add(tuple(cam_izq))
    else:
        print("No se encontró camino 'Seguidor de Pared Izquierda'.")

    # 2. Camino Seguidor de Pared Derecha
    if len(caminos_finales_para_mostrar) < MAX_CAMINOS_A_MOSTRAR: # Esta linea es en realidad innecesaria, pero aniade seguridad al asegurarnos que el 
        # 'MAX_CAMINOS_A_MOSTRAR' no es uno y estemos aniadiendo mas de lo que esperamos. 
        print("\nBuscando camino 'Seguidor de Pared Derecha'...")
        cam_der = encontrar_camino_seguidor_pared(laberinto_num, pos_inicio, pos_fin, alto, ancho, 'derecha')
        if cam_der:
            if tuple(cam_der) not in coords_caminos_vistos:
                print(f"Camino 'Derecha' encontrado (longitud {len(cam_der)}).")
                caminos_finales_para_mostrar.append({"camino": cam_der, "nombre": "Pared Derecha"})
                coords_caminos_vistos.add(tuple(cam_der))
            else:
                print("Camino 'Derecha' es idéntico a uno ya encontrado (probablemente el de izquierda).")
        else:
            print("No se encontró camino 'Seguidor de Pared Derecha'.")

    # 3. Caminos Adicionales con BFS y DFS
    num_caminos_faltantes_total = MAX_CAMINOS_A_MOSTRAR - len(caminos_finales_para_mostrar)

    if num_caminos_faltantes_total > 0:
        # Dividir los caminos restantes aproximadamente por la mitad para BFS y DFS
        num_bfs_necesarios = num_caminos_faltantes_total // 2
        num_dfs_necesarios_inicial = num_caminos_faltantes_total - num_bfs_necesarios

        if num_bfs_necesarios > 0:
            print(f"\nBuscando hasta {num_bfs_necesarios} caminos adicionales con BFS...")
            caminos_bfs_adicionales = encontrar_N_caminos_bfs(
                laberinto_num, pos_inicio, pos_fin, alto, ancho,
                num_bfs_necesarios,
                coords_caminos_vistos  # Pasar el set para evitar duplicados con los ya encontrados
            )
            print(f"BFS encontró {len(caminos_bfs_adicionales)} caminos adicionales nuevos.")
            for idx, c_bfs in enumerate(caminos_bfs_adicionales):
                camino_tuple = tuple(c_bfs)
                # encontrar_N_caminos_bfs ya chequea contra coords_caminos_vistos pasados,
                # pero volvemos a chequear y añadir aquí para asegurar.
                if camino_tuple not in coords_caminos_vistos:
                    caminos_finales_para_mostrar.append({"camino": c_bfs, "nombre": f"BFS Adicional {idx+1}"})
                    coords_caminos_vistos.add(camino_tuple)
        
        # Recalcular cuántos caminos DFS se necesitan, por si BFS encontró menos de lo esperado
        # o si originalmente no se necesitaban caminos BFS.
        num_dfs_necesarios_final = MAX_CAMINOS_A_MOSTRAR - len(caminos_finales_para_mostrar)

        if num_dfs_necesarios_final > 0:
            print(f"\nBuscando hasta {num_dfs_necesarios_final} caminos adicionales con DFS...")
            caminos_dfs_adicionales = encontrar_N_caminos_dfs(
                laberinto_num, pos_inicio, pos_fin, alto, ancho,
                num_dfs_necesarios_final,
                coords_caminos_vistos  # Pasar el set para evitar duplicados
            )
            print(f"DFS encontró {len(caminos_dfs_adicionales)} caminos adicionales nuevos.")
            for idx, c_dfs in enumerate(caminos_dfs_adicionales):
                camino_tuple = tuple(c_dfs)
                if camino_tuple not in coords_caminos_vistos:
                     caminos_finales_para_mostrar.append({"camino": c_dfs, "nombre": f"DFS Adicional {idx+1}"})
                     coords_caminos_vistos.add(camino_tuple)

    if not caminos_finales_para_mostrar:
        print("\nNo se encontró ningún camino para visualizar.")
    else:
        print(f"\n--- Visualización de Grafos para {len(caminos_finales_para_mostrar)} Caminos Seleccionados ---")
        
        if plt.get_backend():
             print(f"Se generarán {len(caminos_finales_para_mostrar)} grafos. Cierra las ventanas de gráficos para finalizar.")
        else:
             print("Advertencia: No se detectó un backend de GUI para Matplotlib. Los gráficos podrían no mostrarse interactivamente.")

        for i, data_camino_info in enumerate(caminos_finales_para_mostrar):
            camino_actual = data_camino_info["camino"]
            nombre_del_camino = data_camino_info["nombre"]
            
            print(f"\n--- {i+1}. Procesando: {nombre_del_camino} (Longitud: {len(camino_actual)} pasos) ---")
            
            # Estas 3 lineas de codigo comentadas abajo sirven para ver el camino del laberinto con asteriscos en la terminal, en caso de que 
            # queramos visualizarlo

            #lab_con_camino_marcado_str = visualizar_camino_en_laberinto(laberinto_usuario_str, camino_actual)
            #for linea_viz in lab_con_camino_marcado_str:
            #   print(linea_viz)

            # Estas instrucciones del camino van a ser las mas utiles al momento de hacer que el carrito realmente resuelva el laberinto. 
            # Las "L" y "R" representan giros (sobre el eje) a la izquierda y derecha respectivamente, mientras que "F" es la cantidad de "casillas"
            # que el carrito debe avanzar para tomar ese giro. 
            instrucciones_camino = convertir_camino_a_instrucciones(camino_actual)
            print(f"Instrucciones: {instrucciones_camino}")

            G_camino_actual, pos_layout_actual, edge_labels_actual = camino_a_grafo_ponderado(camino_actual)
            
            if G_camino_actual.nodes():
                titulo_del_grafo = f"Grafo: {nombre_del_camino} (Pasos: {len(camino_actual)})"
                print(f"Mostrando grafo: {titulo_del_grafo}")
                visualizar_grafo_de_camino(G_camino_actual, pos_layout_actual, edge_labels_actual, titulo_del_grafo)
            else:
                print(f"No se pudo generar un grafo para: {nombre_del_camino}.")
        
        if caminos_finales_para_mostrar and plt.get_backend():
             print("\nTodas las visualizaciones de grafos han sido preparadas.")
             print("Mostrando todas las ventanas de gráficos. Ciérralas para que el script termine.")
             plt.show() 
        elif not plt.get_backend():
             print("Script finalizado. No se pudieron mostrar gráficos debido a la falta de backend de Matplotlib.")
        else:
            print("Script finalizado.")

except ValueError as e_val:
    print(f"Error de Valor: {e_val}")
    traceback.print_exc()
except Exception as e_gen:
    print(f"Ocurrió un error inesperado: {e_gen}")
    traceback.print_exc()