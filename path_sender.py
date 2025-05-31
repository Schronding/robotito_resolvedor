import serial
import time

def enviar_instrucciones(arduino_serial, instrucciones_str):
    if not arduino_serial.isOpen():
        print("La conexión serial no está abierta.")
        return

    print(f"Enviando instrucciones: {instrucciones_str}")
    arduino_serial.write((instrucciones_str + '\n').encode('utf-8'))
    
    estimated_time_ms = 0
    tiempo_de_espera_total = 0

    if instrucciones_str.upper() == "!E": # Convertir a mayúsculas por si acaso
        print("Comando !E detectado, usando tiempo de espera extendido para Arduino.")
        tiempo_de_espera_total = 20.0 # Esperar 20 segundos (ajusta si tu secuencia es más larga)
    elif instrucciones_str.upper() == "!C":
        print("Comando !C detectado, usando tiempo de espera corto.")
        tiempo_de_espera_total = 3.0 # 3 segundos deberían ser suficientes
    else: # Para !Sxxxx y otros comandos
        num_comandos = 0
        # Si es un comando de guardado como !SFFRF, calculamos para la parte FFRF
        if instrucciones_str.upper().startswith("!S") and len(instrucciones_str) > 2:
            comandos_reales = instrucciones_str[2:]
        else:
            comandos_reales = instrucciones_str

        for i, char_command in enumerate(comandos_reales):
            if char_command.upper() == 'F':
                estimated_time_ms += 500 
            elif char_command.upper() == 'R' or char_command.upper() == 'L':
                estimated_time_ms += 1500
            
            if i < len(comandos_reales) - 1:
                estimated_time_ms += 200 # PAUSA_ENTRE_COMANDOS
        
        estimated_time_s = estimated_time_ms / 1000.0
        print(f"Tiempo estimado de ejecución en Arduino (para '{comandos_reales}'): {estimated_time_s:.2f} segundos.")
        tiempo_de_espera_total = estimated_time_s + 4.0 # 4 segundos de búfer

    print(f"Esperando {tiempo_de_espera_total:.2f} segundos para el feedback de Arduino...")
    start_time = time.time()
    
    while time.time() - start_time < tiempo_de_espera_total:
        if arduino_serial.in_waiting > 0:
            try:
                respuesta = arduino_serial.readline().decode('utf-8', errors='ignore').rstrip()
                print(f"Arduino: {respuesta}")
            except Exception as e:
                print(f"Error leyendo de Arduino: {e}")
        time.sleep(0.05)
    
    print("Lectura de feedback de Arduino finalizada (o tiempo de espera agotado).")

if __name__ == "__main__":
    arduino_conn = None
    try:
        # Reemplaza '/dev/ttyUSB0' con tu puerto correcto si es diferente
        # El timeout es importante para que readline no bloquee indefinidamente si no hay datos
        arduino_conn = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
        print(f"Conectado a Arduino en {arduino_conn.name}.")

        # Esperar a que Arduino se reinicie después de abrir el puerto serial.
        # Esto es común. El IDE de Arduino también lo hace.
        print("Esperando 2 segundos para que Arduino se inicialice...")
        time.sleep(2) 

        # Limpiar buffer de entrada leyendo cualquier mensaje inicial de Arduino (como "Arduino listo...")
        while arduino_conn.in_waiting > 0:
            init_msg = arduino_conn.readline().decode('utf-8', errors='ignore').rstrip()
            print(f"Arduino (inicio): {init_msg}")

    except serial.SerialException as e:
        print(f"Error al conectar con Arduino: {e}")
        exit()

    if arduino_conn and arduino_conn.isOpen():
        # Secuencia de prueba
        # secuencia_movimientos = "F" # Prueba con un solo comando primero
        secuencia_movimientos = "!SFFRFLFFFLR" #!SFFRFLFFFLR
        enviar_instrucciones(arduino_conn, secuencia_movimientos)

        arduino_conn.close()
        print("Conexión serial cerrada por Python.")
    else:
        print("No se pudo establecer la conexión con Arduino.")