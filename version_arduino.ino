#include <EEPROM.h> // Incluir la librería para la EEPROM

// Definición de pines para el controlador L298N
const int ENA = 3; 
const int IN1 = 4; 
const int IN2 = 5;
const int ENB = 11;
const int IN3 = 12;
const int IN4 = 13;

// Constantes de movimiento
const int VELOCIDAD_AVANCE = 150;
const int VELOCIDAD_GIRO = 120;
const int TIEMPO_GIRO_90_GRADOS = 1500; // Tiempo para un giro de 90 grados
const int TIEMPO_AVANCE_F = 1650;       // Tiempo para avanzar con comando 'F' (0.5 segundos)
const int PAUSA_ENTRE_COMANDOS = 200;  // Pausa entre la finalización de un comando y el inicio del siguiente

// Direcciones y configuración para la EEPROM
const int EEPROM_ADDR_FLAG = 0;          // Dirección para el flag de validez ('V')
const int EEPROM_ADDR_LENGTH = 1;        // Dirección para guardar la longitud de la cadena
const int EEPROM_ADDR_STRING_START = 2;  // Dirección donde comienza la cadena de comandos
const int MAX_COMMAND_LENGTH = 50;       // Longitud máxima de la cadena de comandos a guardar en EEPROM

String comandosParaEjecutarRAM = ""; // Variable para almacenar comandos temporalmente si es necesario

// --- Funciones de Control de Motores ---
void moverAdelante(int velocidad) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, velocidad);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, velocidad);
    Serial.println("Moviendo Adelante");
}

void girarIzquierda(int velocidad) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, velocidad);   
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, velocidad);  
    Serial.println("Girando Derecha (sobre eje)");
}

void girarDerecha(int velocidad) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, velocidad); 
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, velocidad); 
    Serial.println("Girando Izquierda (sobre eje)");
}

void detenerMotores() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
    Serial.println("Motores Detenidos");
}

// --- Función para Ejecutar la Secuencia de Instrucciones ---
void ejecutarInstrucciones(String instrucciones) {
    if (instrucciones.length() == 0) {
        Serial.println("No hay instrucciones para ejecutar.");
        return;
    }
    Serial.print("Ejecutando secuencia: "); Serial.println(instrucciones);
    for (int i = 0; i < instrucciones.length(); i++) {
        char instruccion = instrucciones.charAt(i);
        Serial.print("Procesando: "); Serial.println(instruccion);

        switch (instruccion) {
            case 'F':
                moverAdelante(VELOCIDAD_AVANCE);
                delay(TIEMPO_AVANCE_F);
                detenerMotores();
                break;
            case 'R':
                girarDerecha(VELOCIDAD_GIRO);
                delay(TIEMPO_GIRO_90_GRADOS);
                detenerMotores();
                break;
            case 'L':
                girarIzquierda(VELOCIDAD_GIRO);
                delay(TIEMPO_GIRO_90_GRADOS);
                detenerMotores();
                break;
            default:
                Serial.print("Instruccion desconocida: '"); Serial.print(instruccion); Serial.println("'");
                break;
        }
        
        // Pausa entre comandos, excepto después del último
        if (i < instrucciones.length() - 1) {
             delay(PAUSA_ENTRE_COMANDOS);
        }
    }
    Serial.println("Secuencia de instrucciones completada.");
}

// --- Funciones para Manejo de EEPROM ---
void guardarStringEnEEPROM(const String& str) {
    if (str.length() == 0 || str.length() > MAX_COMMAND_LENGTH) {
        Serial.println("Error: Cadena vacía o demasiado larga para EEPROM.");
        EEPROM.write(EEPROM_ADDR_FLAG, 0); // Marcar como no válido
        return;
    }
    EEPROM.write(EEPROM_ADDR_FLAG, 'V'); // 'V' para indicar datos válidos
    EEPROM.write(EEPROM_ADDR_LENGTH, str.length());
    for (int i = 0; i < str.length(); i++) {
        EEPROM.write(EEPROM_ADDR_STRING_START + i, str.charAt(i));
    }
    // En Arduino AVR (Uno, Nano, Mega), EEPROM.commit() no es necesario.
    // Se necesita en ESP32/ESP8266: // EEPROM.commit(); 
    Serial.print("Guardado en EEPROM: "); Serial.println(str);
}

String leerStringDesdeEEPROM() {
    if (EEPROM.read(EEPROM_ADDR_FLAG) != 'V') {
        // Serial.println("Flag de EEPROM no válido. No hay datos guardados.");
        return ""; 
    }
    int len = EEPROM.read(EEPROM_ADDR_LENGTH);
    if (len == 0 || len > MAX_COMMAND_LENGTH) {
      // Serial.println("Longitud en EEPROM inválida o cero.");
      return "";
    }
    char data[len + 1]; // +1 para el terminador null
    for (int i = 0; i < len; i++) {
        data[i] = EEPROM.read(EEPROM_ADDR_STRING_START + i);
    }
    data[len] = '\0'; // Añadir terminador null para convertir a String
    return String(data);
}

void borrarComandosEEPROM() {
    EEPROM.write(EEPROM_ADDR_FLAG, 0); // Cualquier valor que no sea 'V' invalida los datos
    EEPROM.write(EEPROM_ADDR_LENGTH, 0);
    // En Arduino AVR, EEPROM.commit() no es necesario.
    Serial.println("Comandos borrados de la EEPROM.");
}

// --- Función de Configuración Inicial (Setup) ---
void setup() {
    Serial.begin(9600);

    // Configurar pines de motor como SALIDA
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    detenerMotores(); // Asegurar que los motores estén detenidos al inicio
    
    Serial.println("-------------------------------------");
    Serial.println("Sistema de Carrito con EEPROM Iniciado");
    Serial.println("Comandos disponibles por Serial:");
    Serial.println("  'secuencia' (ej: FFRFLF) -> Ejecutar");
    Serial.println("  '!Ssecuencia' (ej: !SFFRFLF) -> GUARDAR y ejecutar");
    Serial.println("  '!E' -> Ejecutar desde EEPROM");
    Serial.println("  '!C' -> Borrar EEPROM");
    Serial.println("-------------------------------------");

    // Intentar cargar y ejecutar comandos desde EEPROM al inicio
    String comandosGuardados = leerStringDesdeEEPROM();
    if (comandosGuardados.length() > 0) {
        Serial.print("Comandos encontrados en EEPROM: "); Serial.println(comandosGuardados);
        int retrasoInicioSegundos = 5; // Retraso en segundos
        Serial.print("Esperando "); Serial.print(retrasoInicioSegundos); Serial.println(" segundos antes de ejecutar automáticamente...");
        delay(retrasoInicioSegundos * 1000); // Convertir segundos a milisegundos
        
        Serial.println("Ejecutando automáticamente desde EEPROM...");
        ejecutarInstrucciones(comandosGuardados);
    } else {
        Serial.println("No hay comandos en EEPROM para ejecución automática al inicio.");
    }
    Serial.println("Esperando nuevos comandos por Serial...");
}

// --- Bucle Principal (Loop) ---
void loop() {
    if (Serial.available() > 0) {
        String comandoEntrante = Serial.readStringUntil('\n');
        comandoEntrante.trim(); // Limpiar espacios/newlines
        
        if (comandoEntrante.length() > 0) {
            Serial.print("Comando recibido por Serial: "); Serial.println(comandoEntrante);

            if (comandoEntrante.startsWith("!S")) { 
                // Guardar en EEPROM y ejecutar
                String secuenciaParaGuardar = comandoEntrante.substring(2); // Quita el prefijo "!S"
                if (secuenciaParaGuardar.length() > 0) {
                    guardarStringEnEEPROM(secuenciaParaGuardar);
                    comandosParaEjecutarRAM = secuenciaParaGuardar; // Actualizar para ejecución inmediata
                    ejecutarInstrucciones(comandosParaEjecutarRAM);
                } else {
                    Serial.println("Comando !S sin secuencia para guardar.");
                }
            } else if (comandoEntrante.equalsIgnoreCase("!E")) { 
                // Ejecutar desde EEPROM
                String comandosDeEEPROM = leerStringDesdeEEPROM();
                if (comandosDeEEPROM.length() > 0) {
                    ejecutarInstrucciones(comandosDeEEPROM);
                } else {
                     Serial.println("No hay nada en EEPROM para ejecutar con !E.");
                }
            } else if (comandoEntrante.equalsIgnoreCase("!C")) { 
                // Borrar EEPROM
                borrarComandosEEPROM();
            } else { 
                // Ejecutar directamente la secuencia recibida (no la guarda permanentemente)
                comandosParaEjecutarRAM = comandoEntrante;
                ejecutarInstrucciones(comandosParaEjecutarRAM);
            }
             Serial.println("Esperando nuevos comandos por Serial...");
        }
    }
}