/*
 * ---------------------------------------------------------------------------
 * 
 * TEMA: FREERTOS DUAL CORE + LORA P2P + CSMA/CA
 * AUTOR: MGTI. Saul Isai Soto Ortiz
 * ---------------------------------------------------------------------------
 * * ¿QUÉ VAMOS A APRENDER?
 * 1. MULTITASKING: Usar los 2 núcleos del ESP32. Un núcleo gestiona la radio (urgente)
 * y el otro gestiona los sensores y la pantalla (lógica de negocio).
 * 2. COMUNICACIÓN ENTRE TAREAS: Cómo pasar datos de un núcleo a otro sin que choquen.
 * 3. SIMULACIÓN DE CAPA DE ENLACE: Implementar CSMA/CA (Evitar colisiones de radio)
 * y estructura de paquetes similar a lo que hace LoRaWAN internamente.
 */

#include "heltec.h"

// ==========================================
// 1. CONFIGURACIÓN GENERAL
// ==========================================
#define BAND    915E6  
byte spread_factor = 8; 

// --- Direccionamiento (Capa de Enlace) ---
// Esto simula la "DevAddr" en LoRaWAN.
byte dir_local   = 0xC1; 
byte dir_destino = 0xD3; 

// --- Estructuras de Datos ---
// ¿Por qué una estructura?
// Porque en una "Cola" de FreeRTOS es más ordenado enviar un paquete con
// toda la información junta (el mensaje Y su ID) como si fuera un sobre cerrado.
struct DatosCola {
  String payload;
  int id_mensaje;
};

// --- Variables Compartidas (Recurso Crítico) ---
// Estas variables son peligrosas porque DOS núcleos intentan tocarlas al mismo tiempo.
// Core 0 escribe en ellas cuando recibe radio. Core 1 las lee para mostrarlas en pantalla.
// REQUIEREN PROTECCIÓN (Mutex).
String rx_mensaje = "";
byte   rx_remite = 0;
int    rx_rssi = 0;
bool   rx_nuevo_dato = false;

// --- FreeRTOS Handles (Los "Controladores") ---
TaskHandle_t xHandle_LoRa = NULL;   // Identificador para controlar al Core 0
TaskHandle_t xHandle_Sensor = NULL; // Identificador para controlar al Core 1

// SEMÁFORO (MUTEX): Es como la llave del baño. Solo uno puede tenerla.
// Evita que el Core 1 lea un mensaje a medias mientras el Core 0 lo está escribiendo.
SemaphoreHandle_t xMutex_RX;    

// COLA (QUEUE): Es como una banda transportadora o buzón.
// Permite que el Sensor (Core 1) deje datos para enviar y siga trabajando,
// sin esperar a que la Radio (Core 0) termine de transmitir.
QueueHandle_t xCola_TX;         

#define LED_PIN 25 

// ==========================================
// 2. SETUP (CONFIGURACIÓN DEL SISTEMA)
// ==========================================
void setup() {
  Heltec.begin(true /*Display*/, true /*LoRa*/, true /*Serial*/, true /*PABOOST*/, BAND);
  
  // Inicialización de Pantalla
  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "Iniciando RTOS...");
  Heltec.display->display();

  // Configuración LoRa
  LoRa.setSpreadingFactor(spread_factor);
  LoRa.receive(); // IMPORTANTE: La radio arranca en modo "Escucha"
  
  pinMode(LED_PIN, OUTPUT);

  // --- CREACIÓN DE OBJETOS DEL SISTEMA OPERATIVO (OS) ---
  xMutex_RX = xSemaphoreCreateMutex(); // Creamos la "llave"
  xCola_TX  = xQueueCreate(10, sizeof(DatosCola)); // Creamos el "buzón" para 10 cartas

  // --- LANZAMIENTO DE TAREAS (MULTIPROCESSING) ---
  
  // Tarea 1: GESTOR DE RADIO (Core 0)
  // Prioridad 2 (ALTA): La radio es sensible al tiempo, no debe interrumpirse.
  xTaskCreatePinnedToCore(
    TareaLoRa_Code,   // Función a ejecutar
    "Radio_Task",     // Nombre para depuración
    4096,             // Memoria asignada (Stack)
    NULL,             // Parámetros
    2,                // Prioridad
    &xHandle_LoRa,    // Handle
    0);               // ** PINNED TO CORE 0 **

  // Tarea 2: APLICACIÓN / SENSOR (Core 1)
  // Prioridad 1 (NORMAL): Si se atrasa unos milisegundos, no pasa nada.
  xTaskCreatePinnedToCore(
    TareaSensor_Code, 
    "App_Task", 
    4096, 
    NULL, 
    1, 
    &xHandle_Sensor, 
    1);               // ** PINNED TO CORE 1 **
    
  Serial.println("--- SISTEMA DUAL CORE INICIADO ---");
  Serial.println("Core 0: Dedicado a LoRa | Core 1: Dedicado a Sensores");
}

// ==========================================
// 3. LOOP (CONTROLADOR DE SISTEMA)
// ==========================================
// En RTOS, el loop() pierde protagonismo. Aquí lo usamos solo
// como interfaz de usuario para pausar/reanudar tareas.
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    // Ejemplo de control de tareas en tiempo real
    if (c == 'P' || c == 'p') {
      Serial.println("CMD: Pausando tarea del Sensor (Core 1)...");
      vTaskSuspend(xHandle_Sensor); // Congela la lógica, pero la radio sigue funcionando
    }
    if (c == 'R' || c == 'r') {
      Serial.println("CMD: Reanudando tarea del Sensor...");
      vTaskResume(xHandle_Sensor);  
    }
  }
  vTaskDelay(100); // Delay necesario para no saturar al Watchdog Timer
}

// ==========================================
// 4. TAREA CORE 0: PROTOCOLO DE RADIO (DRIVER)
// ==========================================
// Esta tarea actúa como un "Servicio de Fondo". Siempre está escuchando.
// Solo transmite si encuentra algo en la Cola de Salida.
void TareaLoRa_Code(void * parameter) {
  DatosCola paqueteSaliente; // Variable temporal para sacar datos del buzón
  
  for(;;) { // Bucle infinito de la tarea
    
    // -------------------------------------------------
    // A. GESTIÓN DE RECEPCIÓN (RX) - "Escuchar siempre"
    // -------------------------------------------------
    // La radio verifica si llegó algo por el aire.
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      procesar_recepcion(packetSize); // Desencapsula y valida (ver abajo)
    }

    // -------------------------------------------------
    // B. GESTIÓN DE TRANSMISIÓN (TX) - "Enviar si hay pedido"
    // -------------------------------------------------
    // xQueueReceive verifica el buzón. 
    // El '0' indica TIEMPO DE ESPERA CERO. Si no hay nada, sigue de largo (No bloquea).
    if (xQueueReceive(xCola_TX, &paqueteSaliente, 0) == pdTRUE) {
      
      // --- LÓGICA CSMA (Carrier Sense Multiple Access) ---
      // Antes de hablar, escuchamos. Si el canal está ocupado (RSSI alto), esperamos.
      // LoRa.beginPacket() devuelve 0 si la radio está ocupada transmitiendo o recibiendo.
      while (LoRa.beginPacket() == 0) { 
        vTaskDelay(10); // Espera pasiva (libera CPU)
      }
      
      // --- ENCAPSULAMIENTO TIPO LoRaWAN (Manual) ---
      LoRa.beginPacket();
      LoRa.write(dir_destino);           // Header: ¿A quién?
      LoRa.write(dir_local);             // Header: ¿De quién?
      LoRa.write((byte)paqueteSaliente.id_mensaje); // Header: ID (Frame Counter)
      LoRa.write((byte)paqueteSaliente.payload.length()); // Header: Largo Payload
      LoRa.print(paqueteSaliente.payload); // Payload: Datos del sensor
      LoRa.endPacket();
      
      // Inmediatamente volvemos a escuchar (Receive Window)
      LoRa.receive(); 
      
      // Feedback visual hardware
      digitalWrite(LED_PIN, HIGH); vTaskDelay(100); digitalWrite(LED_PIN, LOW);
      Serial.print("[Core 0] TX Enviado ID:"); Serial.println(paqueteSaliente.id_mensaje);
    }

    // Pequeño descanso para evitar que el Watchdog crea que la tarea se colgó
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// ==========================================
// 5. TAREA CORE 1: APLICACIÓN (SENSOR + UI)
// ==========================================
// Esta tarea simula el comportamiento "inteligente" del dispositivo.
// Lee sensores, decide cuándo enviar y actualiza la interfaz.
void TareaSensor_Code(void * parameter) {
  String estadoSensor = "ON";
  int contador_global = 0;
  DatosCola datosParaEnviar; // Estructura para llenar y meter al buzón

  for(;;) {
    // 1. LECTURA SENSOR (Aquí iría el código DHT11/22 real)
    estadoSensor = (estadoSensor == "ON") ? "OFF" : "ON";
    
    // 2. PREPARAR PAQUETE PARA EL CORE 0
    datosParaEnviar.payload = estadoSensor;
    datosParaEnviar.id_mensaje = contador_global;
    
    // Meter al buzón (xCola_TX). 
    // portMAX_DELAY significa: "Si el buzón está lleno, espérate aquí hasta que haya espacio".
    xQueueSend(xCola_TX, &datosParaEnviar, portMAX_DELAY);
    contador_global++;

    // 3. ACTUALIZAR PANTALLA
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "Core 1: Generando ID " + String(contador_global));
    
    // --- ZONA CRÍTICA (Lectura de datos compartidos) ---
    // Pedimos la "llave" (Mutex) para leer los datos que dejó el Core 0.
    // Esperamos máximo 100 ticks si la llave está ocupada.
    if (xSemaphoreTake(xMutex_RX, (TickType_t)100) == pdTRUE) {
      if (rx_nuevo_dato) {
        Heltec.display->drawString(0, 20, "RX de: 0x" + String(rx_remite, HEX));
        Heltec.display->drawString(0, 30, "Msg: " + rx_mensaje);
        Heltec.display->drawString(0, 40, "RSSI: " + String(rx_rssi) + " dBm");
      } else {
        Heltec.display->drawString(0, 20, "Esperando RX...");
      }
      xSemaphoreGive(xMutex_RX); // ¡Importante! Devolver la llave.
    }
    Heltec.display->display();

    // 4. CA (COLLISION AVOIDANCE) - JITTER
    // En lugar de enviar cada 6 segundos exactos (lo que causaría colisiones si
    // todos los alumnos prenden las placas a la vez), añadimos aleatoriedad.
    // Esto es fundamental en redes LoRaWAN reales para evitar saturación del Gateway.
    long tiempo_wait = 6000 + random(3000);
    vTaskDelay(tiempo_wait / portTICK_PERIOD_MS);
  }
}

// ==========================================
// 6. LÓGICA DE RECEPCIÓN (HELPER)
// ==========================================
/*
 * PROPÓSITO:
 * Esta función actúa como el "Portero" del sistema. 
 * Recibe una trama cruda de bytes, la "desempaqueta", verifica que sea válida 
 * y la pasa de forma segura al área de memoria compartida.
 */
void procesar_recepcion(int packetSize) {
  // Si el paquete está vacío, no perdemos tiempo.
  if (packetSize == 0) return;

  // -----------------------------------------------------------
  // FASE 1: DESERIALIZACIÓN (ABRIR EL SOBRE)
  // -----------------------------------------------------------
  // Los datos llegan en fila india (Stream). Debemos leerlos EN EL MISMO ORDEN
  // en que fueron empaquetados por el emisor.
  // Estructura esperada: [DESTINO] [REMITENTE] [ID] [LARGO] [PAYLOAD...]

  String paq_rcb = "";
  
  // Leemos los 4 bytes del encabezado (Header)
  byte d_envio  = LoRa.read(); // Byte 1: Dirección Destino (¿Para quién es?) Ej: 0xC1
  byte d_remite = LoRa.read(); // Byte 2: Dirección Remitente (¿Quién lo mandó?) Ej: 0xD3
  byte id_msg   = LoRa.read(); // Byte 3: ID del mensaje (Para control de duplicados)
  byte len_msg  = LoRa.read(); // Byte 4: Longitud declarada del mensaje. Ej: 4 bytes ("HOLA")

  // -----------------------------------------------------------
  // FASE 2: RECONSTRUCCIÓN DEL PAYLOAD (LEER LA CARTA)
  // -----------------------------------------------------------
  // Todo lo que queda en el buffer después del encabezado es el mensaje real.
  while (LoRa.available()) {
    paq_rcb += (char)LoRa.read(); // Concatenamos carácter por carácter: "H"+"O"+"L"+"A"
  }

  // -----------------------------------------------------------
  // FASE 3: CAPA DE ENLACE (VALIDACIONES DE SEGURIDAD)
  // -----------------------------------------------------------
  
  // A. VALIDACIÓN DE INTEGRIDAD (Checksum Simple)
  // Comparamos la longitud que "decía" el encabezado (len_msg) contra lo que realmente llegó.
  // Ejemplo de Fallo: El encabezado dice 4 bytes, pero por ruido solo llegaron "HO" (2 bytes).
  if (paq_rcb.length() != len_msg) {
    Serial.println("[RX Error] Paquete corrupto (Longitud incompleta o ruido)");
    return; // Descartamos el paquete inmediatamente.
  }

  // B. FILTRADO DE DIRECCIONES (Address Filtering)
  // Verificamos si el mensaje es para nosotros (Unicast) o para todos (Broadcast).
  // dir_local: Mi dirección (Ej. 0xC1).
  // 0xFF: Dirección universal de Broadcast.
  if (d_envio != dir_local && d_envio != 0xFF) {
    // Si d_envio es 0xB5 (otro equipo), lo ignoramos silenciosamente.
    Serial.println("[RX Ignorado] El paquete no es para este nodo");
    return;
  }

  // -----------------------------------------------------------
  // FASE 4: ZONA CRÍTICA (MUTEX / SEMÁFORO)
  // -----------------------------------------------------------
  /* * PROBLEMA: El Core 1 (Pantalla) puede intentar leer 'rx_mensaje' justo en el milisegundo
   * en que el Core 0 (Radio) lo está escribiendo. Esto causaría datos basura en pantalla.
   * * SOLUCIÓN: Usamos un Mutex (Mutual Exclusion). Es como la "Llave del Baño".
   * Solo quien tiene la llave puede entrar a modificar/leer las variables globales.
   */

  // xSemaphoreTake: Intentamos tomar la llave. Esperamos máximo 10 ticks si está ocupada.
  if (xSemaphoreTake(xMutex_RX, (TickType_t)10) == pdTRUE) {
    
    // --- INICIO DE ZONA SEGURA ---
    // Aquí adentro NADIE más puede tocar estas variables.
    
    rx_mensaje = paq_rcb;          // Guardamos el mensaje validado "HOLA"
    rx_remite  = d_remite;         // Guardamos quién lo envió (0xD3)
    rx_rssi    = LoRa.packetRssi();// Guardamos la potencia de señal actual (Ej: -85dBm)
    rx_nuevo_dato = true;          // Levantamos la bandera para avisar al Core 1
    
    // --- FIN DE ZONA SEGURA ---
    
    xSemaphoreGive(xMutex_RX); // ¡MUY IMPORTANTE! Devolver la llave para que el Core 1 pueda usarla.
    
    // Feedback de depuración
    Serial.print("[Core 0] RX Válido de 0x"); Serial.print(d_remite, HEX);
    Serial.print(" | RSSI: "); Serial.println(rx_rssi);
  } else {
    Serial.println("[Error OS] No se pudo obtener el Mutex (Sistema ocupado)");
  }
}
