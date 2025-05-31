import collections
import networkx as nx
import matplotlib.pyplot as plt
import traceback # Para imprimir errores detallados
import serial    # Para la comunicación con Arduino
import time      # Para pausas y timeouts

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
NEIGHBOR_ORDER_global = ['E', 'S', 'W', 'N'] # Preferencia para DFS y BFS estándar

# Mapeos para algoritmos basados en dirección indexada (0:N, 1:E, 2:S, 3:W)
DIR_TO_IDX = {(-1,0):0, (0,1):1, (1,0):2, (0,-1):3} 
IDX_TO_DR_DC = {0:(-1,0), 1:(0,1), 2:(1,0), 3:(0,-1)}

# --- Representacion grafica de nuestro laberinto real ---
# (Tomado de path_sender.py, asumiendo que esta es la versión definitiva que quieres usar)
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

# --- Funciones de Procesamiento del Laberinto y Búsqueda de Caminos ---
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
                print(f"Advertencia: Caracter '{char}' no reconocido en ({r_idx},{c_idx}). Tratado como muro.")
                fila_num.append(WALL)
        mapa_numerico.append(fila_num)

    if not pos_inicio:
        raise ValueError("No se encontró el punto de inicio 'S' en el laberinto.")
    if not pos_fin:
        raise ValueError("No se encontró el punto de fin 'E' en el laberinto.")
    return mapa_numerico, pos_inicio, pos_fin, height, width

def encontrar_camino_seguidor_pared(laberinto_num, inicio, fin, height, width, tipo_seguidor):
    camino_actual = [inicio]
    r, c = inicio
    dir_mirada_idx = -1
    for dr_char_init in NEIGHBOR_ORDER_global: 
        dr_init, dc_init = DIRECTIONS_map[dr_char_init]
        nr_init, nc_init = r + dr_init, c + dc_init
        if 0 <= nr_init < height and 0 <= nc_init < width and laberinto_num[nr_init][nc_init] != WALL:
            dir_mirada_idx = DIR_TO_IDX[(dr_init, dc_init)]
            r, c = nr_init, nc_init
            camino_actual.append((r,c))
            break 
    if dir_mirada_idx == -1: return None
    max_pasos_simulacion = height * width * 2
    pasos_realizados = 0 
    while (r, c) != fin and pasos_realizados < max_pasos_simulacion:
        pasos_realizados += 1
        if tipo_seguidor == 'izquierda':
            idx_orden_prueba = [(dir_mirada_idx - 1 + 4) % 4, dir_mirada_idx, (dir_mirada_idx + 1) % 4, (dir_mirada_idx + 2) % 4]
        else: 
            idx_orden_prueba = [(dir_mirada_idx + 1) % 4, dir_mirada_idx, (dir_mirada_idx - 1 + 4) % 4, (dir_mirada_idx + 2) % 4]
        movimiento_efectuado_este_paso = False
        for idx_nueva_direccion_mirada in idx_orden_prueba:
            dr_new, dc_new = IDX_TO_DR_DC[idx_nueva_direccion_mirada]
            nr_new, nc_new = r + dr_new, c + dc_new
            if 0 <= nr_new < height and 0 <= nc_new < width and laberinto_num[nr_new][nc_new] != WALL:
                r, c = nr_new, nc_new
                camino_actual.append((r,c))
                dir_mirada_idx = idx_nueva_direccion_mirada
                movimiento_efectuado_este_paso = True
                break 
        if not movimiento_efectuado_este_paso: return None 
    if (r,c) == fin: return camino_actual
    else: return None

def encontrar_N_caminos_dfs(laberinto_num, inicio, fin, height, width, N_caminos_max, caminos_existentes_coords_set):
    caminos_encontrados_dfs = []
    laberinto_transitable = [list(fila) for fila in laberinto_num]
    if laberinto_transitable[inicio[0]][inicio[1]] == START: laberinto_transitable[inicio[0]][inicio[1]] = PATH
    if laberinto_transitable[fin[0]][fin[1]] == END: laberinto_transitable[fin[0]][fin[1]] = PATH
    pila = collections.deque([(inicio, [inicio])]) 
    while pila:
        (r_curr, c_curr), camino_parcial = pila.pop()
        if (r_curr, c_curr) == fin:
            camino_tuple = tuple(camino_parcial)
            if camino_tuple not in caminos_existentes_coords_set:
                caminos_encontrados_dfs.append(list(camino_parcial))
                if len(caminos_encontrados_dfs) >= N_caminos_max: return caminos_encontrados_dfs
            continue 
        for dr_char in NEIGHBOR_ORDER_global:
            dr, dc = DIRECTIONS_map[dr_char]
            nr_next, nc_next = r_curr + dr, c_curr + dc
            if 0 <= nr_next < height and 0 <= nc_next < width and \
               laberinto_transitable[nr_next][nc_next] != WALL and \
               (nr_next, nc_next) not in camino_parcial: 
                nuevo_camino_parcial = list(camino_parcial); nuevo_camino_parcial.append((nr_next, nc_next))
                pila.append(((nr_next, nc_next), nuevo_camino_parcial))
    return caminos_encontrados_dfs

def encontrar_N_caminos_bfs(laberinto_num, inicio, fin, height, width, N_caminos_max, caminos_existentes_coords_set):
    caminos_encontrados_bfs = []
    laberinto_transitable = [list(fila) for fila in laberinto_num]
    if laberinto_transitable[inicio[0]][inicio[1]] == START: laberinto_transitable[inicio[0]][inicio[1]] = PATH
    if laberinto_transitable[fin[0]][fin[1]] == END: laberinto_transitable[fin[0]][fin[1]] = PATH
    queue = collections.deque([(inicio, [inicio])])
    while queue:
        (r_curr, c_curr), camino_parcial = queue.popleft()
        for dr_char in NEIGHBOR_ORDER_global:
            dr, dc = DIRECTIONS_map[dr_char]
            nr_next, nc_next = r_curr + dr, c_curr + dc
            if 0 <= nr_next < height and 0 <= nc_next < width and \
               laberinto_transitable[nr_next][nc_next] != WALL and \
               (nr_next, nc_next) not in camino_parcial:
                nuevo_camino_parcial = list(camino_parcial); nuevo_camino_parcial.append((nr_next, nc_next))
                if (nr_next, nc_next) == fin:
                    camino_tuple = tuple(nuevo_camino_parcial)
                    if camino_tuple not in caminos_existentes_coords_set:
                        caminos_encontrados_bfs.append(nuevo_camino_parcial)
                        if len(caminos_encontrados_bfs) >= N_caminos_max: return caminos_encontrados_bfs
                else:
                    queue.append(((nr_next, nc_next), nuevo_camino_parcial))
    return caminos_encontrados_bfs 

def convertir_camino_a_instrucciones(camino_coordenadas):
    if not camino_coordenadas or len(camino_coordenadas) < 2: return ""
    instrucciones = []
    dr_actual, dc_actual = (camino_coordenadas[1][0] - camino_coordenadas[0][0], camino_coordenadas[1][1] - camino_coordenadas[0][1])
    if (dr_actual, dc_actual) not in DIR_TO_IDX: raise ValueError(f"Mov. inicial inválido: {camino_coordenadas[0]}->{camino_coordenadas[1]}")
    dir_idx_actual = DIR_TO_IDX[(dr_actual, dc_actual)]; instrucciones.append('F')
    for i in range(1, len(camino_coordenadas) - 1):
        dr_nuevo, dc_nuevo = (camino_coordenadas[i+1][0] - camino_coordenadas[i][0], camino_coordenadas[i+1][1] - camino_coordenadas[i][1])
        if (dr_nuevo, dc_nuevo) not in DIR_TO_IDX: raise ValueError(f"Mov. inválido: {camino_coordenadas[i]}->{camino_coordenadas[i+1]}")
        dir_idx_nuevo = DIR_TO_IDX[(dr_nuevo, dc_nuevo)]
        if dir_idx_nuevo == dir_idx_actual: instrucciones.append('F')
        elif dir_idx_nuevo == (dir_idx_actual + 1) % 4: instrucciones.extend(['R', 'F'])
        elif dir_idx_nuevo == (dir_idx_actual - 1 + 4) % 4: instrucciones.extend(['L', 'F'])
        else: instrucciones.extend(['R', 'R', 'F'])
        dir_idx_actual = dir_idx_nuevo
    return "".join(instrucciones)

# --- Funciones de Visualización y Grafo ---
def visualizar_camino_en_laberinto(laberinto_str_list, camino_coordenadas):
    # ... (sin cambios respecto a tu versión, asumiendo que está correcta)
    if not camino_coordenadas: return ["Laberinto original"] + laberinto_str_list
    lab_visual = [list(fila) for fila in laberinto_str_list]
    for r_coord, c_coord in camino_coordenadas:
        if lab_visual[r_coord][c_coord] != START_CHAR and lab_visual[r_coord][c_coord] != END_CHAR:
            lab_visual[r_coord][c_coord] = '*'
    if camino_coordenadas: 
        lab_visual[camino_coordenadas[0][0]][camino_coordenadas[0][1]] = START_CHAR
        lab_visual[camino_coordenadas[-1][0]][camino_coordenadas[-1][1]] = END_CHAR
    return ["Laberinto con camino marcado (*):"] + ["".join(fila) for fila in lab_visual]

def camino_a_grafo_ponderado(camino_coordenadas):
    # ... (sin cambios)
    if not camino_coordenadas or len(camino_coordenadas) < 2: return nx.Graph(), {}, {}
    G = nx.Graph(); pos_layout = {}; edge_labels = {}
    nodo_grafo_actual = camino_coordenadas[0]
    G.add_node(nodo_grafo_actual)
    pos_layout[nodo_grafo_actual] = (camino_coordenadas[0][1], -camino_coordenadas[0][0])
    pasos_segmento = 0
    dr_seg, dc_seg = (camino_coordenadas[1][0] - camino_coordenadas[0][0], camino_coordenadas[1][1] - camino_coordenadas[0][1])
    dir_actual_segmento = (dr_seg, dc_seg)
    for i in range(len(camino_coordenadas) - 1):
        pasos_segmento += 1
        coord_sig = camino_coordenadas[i+1]
        es_final = (i + 1 == len(camino_coordenadas) - 1)
        cambio_dir = False
        if not es_final:
            dr_prox, dc_prox = (camino_coordenadas[i+2][0] - coord_sig[0], camino_coordenadas[i+2][1] - coord_sig[1])
            if (dr_prox, dc_prox) != dir_actual_segmento: cambio_dir = True
        if es_final or cambio_dir:
            nuevo_nodo = coord_sig
            G.add_node(nuevo_nodo); pos_layout[nuevo_nodo] = (nuevo_nodo[1], -nuevo_nodo[0])
            G.add_edge(nodo_grafo_actual, nuevo_nodo, weight=pasos_segmento)
            edge_labels[(nodo_grafo_actual, nuevo_nodo)] = pasos_segmento
            nodo_grafo_actual = nuevo_nodo; pasos_segmento = 0
            if not es_final: dir_actual_segmento = (dr_prox, dc_prox)
    return G, pos_layout, edge_labels

def visualizar_grafo_de_camino(G, pos, edge_labels, title):
    # ... (sin cambios, usa num=title)
    if not G.nodes(): print(f"Grafo para '{title}' vacío."); return
    plt.figure(num=title, figsize=(9, 7)) 
    node_labels = {node: f"({node[0]},{node[1]})" for node in G.nodes()}
    nx.draw(G, pos, with_labels=True, labels=node_labels, node_color='lightgreen', node_size=1000, font_size=7, font_weight='normal')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='darkred', font_size=7)
    plt.title(title, fontsize=10) 
    plt.draw(); plt.pause(0.01) 

# --- Función para Enviar Instrucciones a Arduino (de path_sender.py) ---
def enviar_instrucciones(arduino_serial, instrucciones_str):
    """Función para enviar instrucciones al Arduino y manejar el feedback."""
    if not arduino_serial.isOpen():
        print("La conexión serial no está abierta.")
        return

    print(f"Enviando instrucciones: {instrucciones_str}")
    arduino_serial.write((instrucciones_str + '\n').encode('utf-8')) # Añadir terminador de línea
    
    # Estimación de tiempo y manejo de feedback (lógica de path_sender.py)
    estimated_time_ms = 0
    tiempo_de_espera_total = 0

    # Casos especiales para !E y !C
    if instrucciones_str.upper() == "!E":
        print("Comando !E detectado, usando tiempo de espera extendido para Arduino.")
        tiempo_de_espera_total = 20.0 
    elif instrucciones_str.upper() == "!C":
        print("Comando !C detectado, usando tiempo de espera corto.")
        tiempo_de_espera_total = 3.0
    else: 
        # Cálculo para secuencias de movimiento (ej. !SFFRF o FFRF)
        comandos_reales = instrucciones_str
        if instrucciones_str.upper().startswith("!S") and len(instrucciones_str) > 2:
            comandos_reales = instrucciones_str[2:] # Solo la parte de los movimientos

        for i, char_command in enumerate(comandos_reales):
            if char_command.upper() == 'F':
                estimated_time_ms += 500 
            elif char_command.upper() == 'R' or char_command.upper() == 'L':
                estimated_time_ms += 1500
            
            # PAUSA_ENTRE_COMANDOS (si aplica también para el último comando, ajustar)
            if i < len(comandos_reales) - 1: # No añadir pausa después del último comando
                estimated_time_ms += 200 
        
        estimated_time_s = estimated_time_ms / 1000.0
        print(f"Tiempo estimado de ejecución en Arduino (para '{comandos_reales}'): {estimated_time_s:.2f} segundos.")
        # Buffer adicional para comunicación, procesamiento en Arduino, etc.
        tiempo_de_espera_total = estimated_time_s + 4.0 # 4 segundos de búfer general

    print(f"Esperando hasta {tiempo_de_espera_total:.2f} segundos para el feedback de Arduino...")
    start_time = time.time()
    
    # Bucle para leer feedback
    buffer_arduino = ""
    while time.time() - start_time < tiempo_de_espera_total:
        if arduino_serial.in_waiting > 0:
            try:
                # Leer bytes y decodificar, acumulando hasta encontrar un '\n'
                byte_leido = arduino_serial.read(arduino_serial.in_waiting)
                buffer_arduino += byte_leido.decode('utf-8', errors='ignore')
                
                while '\n' in buffer_arduino:
                    respuesta, buffer_arduino = buffer_arduino.split('\n', 1)
                    respuesta = respuesta.rstrip() # Quitar \r si existe
                    if respuesta: # Solo imprimir si no está vacío
                        print(f"Arduino: {respuesta}")
            except Exception as e:
                print(f"Error leyendo de Arduino: {e}")
                # Podríamos romper el bucle aquí si el error es crítico
        time.sleep(0.05) # Pequeña pausa para no saturar CPU
    
    # Imprimir cualquier resto en el buffer que no terminó en \n (si es relevante)
    if buffer_arduino.strip():
        print(f"Arduino (buffer restante): {buffer_arduino.strip()}")

    print("Lectura de feedback de Arduino finalizada (o tiempo de espera agotado).")


# --- Procesamiento Principal ---
if __name__ == "__main__":
    arduino_conn = None # Mover la inicialización aquí para el bloque finally
    caminos_ordenados_por_longitud = [] # Para accederla en la sección de envío

    try:
        print("Parseando laberinto...")
        laberinto_num, pos_inicio, pos_fin, alto, ancho = parse_laberinto(laberinto_real)
        print(f"Laberinto parseado. Inicio: {pos_inicio}, Fin: {pos_fin}, Dimensiones: {alto}x{ancho}")
        
        caminos_finales_para_mostrar = []
        coords_caminos_vistos = set() 
        MAX_CAMINOS_A_MOSTRAR = 6 # Modifica según necesites
        info_caminos_para_ordenar = []

        # 1. Seguidor de Pared Izquierda
        print("\nBuscando camino 'Seguidor de Pared Izquierda'...")
        cam_izq = encontrar_camino_seguidor_pared(laberinto_num, pos_inicio, pos_fin, alto, ancho, 'izquierda')
        if cam_izq:
            print(f"Camino 'Izquierda' encontrado (longitud {len(cam_izq)}).")
            caminos_finales_para_mostrar.append({"camino": cam_izq, "nombre": "Pared Izquierda"})
            coords_caminos_vistos.add(tuple(cam_izq))
        else: print("No se encontró camino 'Seguidor de Pared Izquierda'.")

        # 2. Seguidor de Pared Derecha
        if len(caminos_finales_para_mostrar) < MAX_CAMINOS_A_MOSTRAR:
            print("\nBuscando camino 'Seguidor de Pared Derecha'...")
            cam_der = encontrar_camino_seguidor_pared(laberinto_num, pos_inicio, pos_fin, alto, ancho, 'derecha')
            if cam_der:
                if tuple(cam_der) not in coords_caminos_vistos:
                    print(f"Camino 'Derecha' encontrado (longitud {len(cam_der)}).")
                    caminos_finales_para_mostrar.append({"camino": cam_der, "nombre": "Pared Derecha"})
                    coords_caminos_vistos.add(tuple(cam_der))
                else: print("Camino 'Derecha' es idéntico a uno ya encontrado.")
            else: print("No se encontró camino 'Seguidor de Pared Derecha'.")

        # 3. Caminos Adicionales con BFS y DFS
        num_caminos_faltantes_total = MAX_CAMINOS_A_MOSTRAR - len(caminos_finales_para_mostrar)
        if num_caminos_faltantes_total > 0:
            num_bfs_necesarios = num_caminos_faltantes_total // 2
            if num_bfs_necesarios > 0:
                print(f"\nBuscando hasta {num_bfs_necesarios} caminos adicionales con BFS...")
                caminos_bfs = encontrar_N_caminos_bfs(laberinto_num, pos_inicio, pos_fin, alto, ancho, num_bfs_necesarios, coords_caminos_vistos)
                print(f"BFS encontró {len(caminos_bfs)} caminos adicionales nuevos.")
                for idx, c_bfs in enumerate(caminos_bfs):
                    ct = tuple(c_bfs); 
                    if ct not in coords_caminos_vistos: 
                        caminos_finales_para_mostrar.append({"camino": c_bfs, "nombre": f"BFS Adicional {idx+1}"}); coords_caminos_vistos.add(ct)
            num_dfs_necesarios = MAX_CAMINOS_A_MOSTRAR - len(caminos_finales_para_mostrar)
            if num_dfs_necesarios > 0:
                print(f"\nBuscando hasta {num_dfs_necesarios} caminos adicionales con DFS...")
                caminos_dfs = encontrar_N_caminos_dfs(laberinto_num, pos_inicio, pos_fin, alto, ancho, num_dfs_necesarios, coords_caminos_vistos)
                print(f"DFS encontró {len(caminos_dfs)} caminos adicionales nuevos.")
                for idx, c_dfs in enumerate(caminos_dfs):
                    ct = tuple(c_dfs); 
                    if ct not in coords_caminos_vistos: 
                        caminos_finales_para_mostrar.append({"camino": c_dfs, "nombre": f"DFS Adicional {idx+1}"}); coords_caminos_vistos.add(ct)
        
        if not caminos_finales_para_mostrar:
            print("\nNo se encontró ningún camino para visualizar.")
        else:
            print(f"\n--- Procesamiento de {len(caminos_finales_para_mostrar)} Caminos Seleccionados ---")
            for i, data_camino_info in enumerate(caminos_finales_para_mostrar):
                camino_actual = data_camino_info["camino"]
                nombre_del_camino = data_camino_info["nombre"]
                longitud_camino_actual = len(camino_actual)
                print(f"\n{i+1}. Procesando: {nombre_del_camino} (Celdas: {longitud_camino_actual})")
                instrucciones_camino = convertir_camino_a_instrucciones(camino_actual)
                print(f"   Instrucciones: {instrucciones_camino}")
                info_caminos_para_ordenar.append({
                    "nombre": nombre_del_camino, "longitud": longitud_camino_actual,
                    "instrucciones": instrucciones_camino, "coordenadas": camino_actual
                })
                if plt.get_backend(): # Solo intentar graficar si hay backend
                    G_camino, pos_layout, edge_labels = camino_a_grafo_ponderado(camino_actual)
                    if G_camino.nodes():
                        titulo_grafo = f"Grafo: {nombre_del_camino} (Celdas: {longitud_camino_actual})"
                        visualizar_grafo_de_camino(G_camino, pos_layout, edge_labels, titulo_grafo)
            
            if info_caminos_para_ordenar:
                caminos_ordenados_por_longitud = sorted(info_caminos_para_ordenar, key=lambda x: x["longitud"])
                print("\n\n═══════════════════════════════════════════════════════════════")
                print("  LISTA DE CAMINOS ORDENADOS POR EFICIENCIA (MENOR A MAYOR LONGITUD)  ")
                print("═══════════════════════════════════════════════════════════════")
                for idx, datos in enumerate(caminos_ordenados_por_longitud):
                    print(f"\n{idx}. Camino: {datos['nombre']} (Ranking {idx})") # Ranking 0-based
                    print(f"   Longitud (Celdas): {datos['longitud']}")
                    print(f"   Instrucciones: {datos['instrucciones']}")
                    print("---------------------------------------------------------------")
            else:
                print("\nNo hay caminos procesados para ordenar y enviar.")

            # --- Sección de Envío a Arduino ---
            if caminos_ordenados_por_longitud: # Solo si hay caminos para enviar
                print("\n--- ENVÍO DE INSTRUCCIONES A ARDUINO ---")
                try:
                    # Reemplaza '/dev/ttyUSB0' con tu puerto correcto (ej. 'COM3' en Windows)
                    # Elige el puerto correcto para tu sistema operativo y conexión Arduino.
                    # Puedes listarlos con: python -m serial.tools.list_ports
                    puerto_arduino = '/dev/ttyUSB0' # Ejemplo para Linux, o '/dev/ttyUSB0'
                    # puerto_arduino = 'COM3' # Ejemplo para Windows
                    
                    print(f"Intentando conectar a Arduino en {puerto_arduino}...")
                    arduino_conn = serial.Serial(port=puerto_arduino, baudrate=9600, timeout=1)
                    print(f"Conectado a Arduino en {arduino_conn.name}.")
                    print("Esperando 2 segundos para que Arduino se inicialice...")
                    time.sleep(2) 
                    while arduino_conn.in_waiting > 0: # Limpiar buffer inicial
                        init_msg = arduino_conn.readline().decode('utf-8', errors='ignore').rstrip()
                        if init_msg: print(f"Arduino (inicio): {init_msg}")

                    # (dentro de la sección de Envío a Arduino, después de conectar con el Arduino)
                    while True:
                        try:
                            # Mensaje de entrada actualizado para mayor claridad
                            prompt_message = (
                                f"RANKING (0 a {len(caminos_ordenados_por_longitud)-1}) o '!S<ranking>' (ej: !S0) para GUARDAR ruta,\n"
                                f"'!E' para EJECUTAR memoria, '!C' para CALIBRAR, o 's' para SALIR: "
                            )
                            rank_input_original = input(prompt_message)
                            
                            rank_input_lower = rank_input_original.lower()
                            rank_input_upper = rank_input_original.upper()

                            if rank_input_lower == 's':
                                print("Saliendo del modo de envío de instrucciones.")
                                break
                            
                            if rank_input_upper == '!E' or rank_input_upper == '!C':
                                # Enviar comando especial (!E o !C) directamente
                                print(f"Enviando comando directo al Arduino: {rank_input_upper}")
                                enviar_instrucciones(arduino_conn, rank_input_upper)
                            
                            elif rank_input_upper.startswith("!S") and len(rank_input_upper) > 2:
                                # El usuario ingresó algo como !S0, !S1, etc.
                                rank_str = rank_input_original[2:] # Extraer la parte numérica
                                try:
                                    rank = int(rank_str)
                                    if 0 <= rank < len(caminos_ordenados_por_longitud):
                                        path_to_send = caminos_ordenados_por_longitud[rank]
                                        instrucciones_con_prefijo = "!S" + path_to_send['instrucciones']
                                        print(f"Enviando para GUARDAR en Arduino el camino '{path_to_send['nombre']}' (Ranking {rank}) con comando: {instrucciones_con_prefijo}")
                                        enviar_instrucciones(arduino_conn, instrucciones_con_prefijo)
                                    else:
                                        print(f"Ranking numérico '{rank_str}' fuera de rango. Válidos: 0 a {len(caminos_ordenados_por_longitud)-1}.")
                                except ValueError:
                                    print(f"No se pudo entender el número de ranking en '{rank_input_original}'. Use formato como '!S0', '!S1', etc.")
                            
                            else:
                                # Intentar interpretar la entrada como un número de ranking directo (ej: 0, 1, 2)
                                try:
                                    rank = int(rank_input_original)
                                    if 0 <= rank < len(caminos_ordenados_por_longitud):
                                        path_to_send = caminos_ordenados_por_longitud[rank]
                                        instrucciones_con_prefijo = "!S" + path_to_send['instrucciones']
                                        print(f"Enviando para GUARDAR en Arduino el camino '{path_to_send['nombre']}' (Ranking {rank}) con comando: {instrucciones_con_prefijo}")
                                        enviar_instrucciones(arduino_conn, instrucciones_con_prefijo)
                                    else:
                                        print(f"Ranking inválido. Por favor ingrese un número entre 0 y {len(caminos_ordenados_por_longitud)-1}, o un comando como '!S0', '!E', '!C', 's'.")
                                except ValueError:
                                    # Si no es 's', '!E', '!C', '!S<num>', ni un número, es inválido
                                    print(f"Entrada '{rank_input_original}' no reconocida. Use un ranking (ej: 0), '!S<ranking>' (ej: !S0), '!E', '!C', o 's'.")
                        
                        except Exception as e_send_loop:
                            print(f"Error inesperado en el bucle de envío: {e_send_loop}")
                            traceback.print_exc()
                            # Considerar si se debe romper el bucle o continuar
                
                except serial.SerialException as e_serial:
                    print(f"Error al conectar o comunicar con Arduino ({puerto_arduino}): {e_serial}")
                    print("Asegúrate de que el Arduino esté conectado y el puerto sea el correcto.")
                    print("Puertos seriales disponibles:")
                    try:
                        import serial.tools.list_ports
                        ports = serial.tools.list_ports.comports()
                        for port, desc, hwid in sorted(ports):
                            print(f"- {port}: {desc} [{hwid}]")
                    except ImportError:
                        print("  (No se pudo importar serial.tools.list_ports para listar puertos)")
                    except Exception as e_list_ports:
                        print(f"  (Error al listar puertos: {e_list_ports})")


            if plt.get_backend() and caminos_finales_para_mostrar:
                 print("\nTodas las visualizaciones de grafos han sido preparadas.")
                 print("Mostrando todas las ventanas de gráficos. Ciérralas para que el script termine completamente.")
                 plt.show() 
            elif not plt.get_backend():
                 print("\nScript finalizado. No se pudieron mostrar gráficos debido a la falta de backend de Matplotlib.")
            else:
                print("\nScript finalizado (sin gráficos para mostrar o sin backend).")

    except ValueError as e_val:
        print(f"Error de Valor: {e_val}")
        traceback.print_exc()
    except Exception as e_gen:
        print(f"Ocurrió un error inesperado: {e_gen}")
        traceback.print_exc()
    finally:
        if arduino_conn and arduino_conn.isOpen():
            arduino_conn.close()
            print("Conexión serial con Arduino cerrada por Python.")