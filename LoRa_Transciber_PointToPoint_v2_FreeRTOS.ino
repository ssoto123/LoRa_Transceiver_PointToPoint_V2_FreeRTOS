/*
 * ---------------------------------------------------------------------------
 * ASIGNATURA: SISTEMAS DE SENSORES | MAESTRÍA IOT
 * TEMA: FREERTOS DUAL CORE + LORA P2P + CSMA/CA
 * ---------------------------------------------------------------------------
 * Core 0: Protocolo LoRa (RX continuo + TX cuando hay cola + Carrier Sense)
 * Core 1: Aplicación (Sensor + Jitter Aleatorio + Pantalla)
 */

#include "heltec.h"

// ==========================================
// 1. CONFIGURACIÓN GENERAL
// ==========================================
#define BAND    915E6  
byte spread_factor = 8; 

// --- Direccionamiento ---
byte dir_local   = 0xD3; 
byte dir_destino = 0xC1; 

// --- Estructuras de Datos ---
// Objeto para enviar datos del Core 1 al Core 0
struct DatosCola {
  String payload;
  int id_mensaje;
};

// Variables Compartidas (Protegidas por Mutex) para RX
String rx_mensaje = "";
byte   rx_remite = 0;
int    rx_rssi = 0;
bool   rx_nuevo_dato = false;

// --- FreeRTOS Handles ---
TaskHandle_t xHandle_LoRa = NULL;
TaskHandle_t xHandle_Sensor = NULL;
SemaphoreHandle_t xMutex_RX;    // Protege lectura/escritura de datos recibidos
QueueHandle_t xCola_TX;         // Buzón de salida (Sensor -> Radio)

#define LED_PIN 25 

// ==========================================
// 2. SETUP
// ==========================================
void setup() {
  Heltec.begin(true /*Display*/, true /*LoRa*/, true /*Serial*/, true /*PABOOST*/, BAND);
  
  // OLED Init
  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "Iniciando RTOS...");
  Heltec.display->display();

  // LoRa Init
  LoRa.setSpreadingFactor(spread_factor);
  LoRa.receive(); // Modo escucha inicial
  
  pinMode(LED_PIN, OUTPUT);

  // --- CREAR OBJETOS DEL SISTEMA OPERATIVO ---
  xMutex_RX = xSemaphoreCreateMutex();
  xCola_TX  = xQueueCreate(10, sizeof(DatosCola)); // Cola de 10 mensajes máximo

  // --- LANZAR TAREAS ---
  
  // Tarea 1: RADIO LORA (Core 0 - Alta Prioridad)
  // Se encarga de escuchar SIEMPRE y enviar cuando la cola tenga datos.
  xTaskCreatePinnedToCore(
    TareaLoRa_Code, "Radio_Task", 4096, NULL, 2, &xHandle_LoRa, 0);

  // Tarea 2: SENSOR Y UI (Core 1 - Prioridad Normal)
  // Se encarga de la lógica de negocio, tiempos aleatorios y pantalla.
  xTaskCreatePinnedToCore(
    TareaSensor_Code, "App_Task", 4096, NULL, 1, &xHandle_Sensor, 1);
    
  Serial.println("--- SISTEMA DUAL CORE INICIADO ---");
  Serial.println("Comandos: 'P' (Pausar sensor), 'R' (Reanudar sensor)");
}

// ==========================================
// 3. LOOP (CONTROLADOR)
// ==========================================
void loop() {
  // Solo gestiona comandos de usuario
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'P' || c == 'p') {
      Serial.println("CMD: Pausando generación de datos...");
      vTaskSuspend(xHandle_Sensor); // Pausa Core 1
    }
    if (c == 'R' || c == 'r') {
      Serial.println("CMD: Reanudando generación de datos...");
      vTaskResume(xHandle_Sensor);  // Reanuda Core 1
    }
  }
  vTaskDelay(100); // Libera CPU del loop principal
}

// ==========================================
// 4. TAREA CORE 0: PROTOCOLO DE RADIO
// ==========================================
void TareaLoRa_Code(void * parameter) {
  DatosCola paqueteSaliente;
  
  for(;;) {
    // -------------------------------------------------
    // A. GESTIÓN DE RECEPCIÓN (RX)
    // -------------------------------------------------
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      procesar_recepcion(packetSize); // Función detallada abajo
    }

    // -------------------------------------------------
    // B. GESTIÓN DE TRANSMISIÓN (TX)
    // -------------------------------------------------
    // Revisamos si la otra tarea dejó algo en el buzón.
    // Usamos wait 0 para no bloquear la recepción.
    if (xQueueReceive(xCola_TX, &paqueteSaliente, 0) == pdTRUE) {
      
      // 1. CARRIER SENSE (CSMA Físico)
      // Si la radio está ocupada recibiendo o transmitiendo, esperamos.
      while (LoRa.beginPacket() == 0) { 
        vTaskDelay(10); // Espera no bloqueante
      }
      
      // 2. CONSTRUCCIÓN DEL PAQUETE
      LoRa.beginPacket();
      LoRa.write(dir_destino);           // Byte 1: Destino
      LoRa.write(dir_local);             // Byte 2: Remitente
      LoRa.write((byte)paqueteSaliente.id_mensaje); // Byte 3: ID
      LoRa.write((byte)paqueteSaliente.payload.length()); // Byte 4: Len
      LoRa.print(paqueteSaliente.payload); // Payload
      LoRa.endPacket();
      
      // 3. VUELTA A ESCUCHA
      LoRa.receive(); 
      
      // Feedback visual TX
      digitalWrite(LED_PIN, HIGH); vTaskDelay(100); digitalWrite(LED_PIN, LOW);
      Serial.print("[Core 0] TX ID:"); Serial.println(paqueteSaliente.id_mensaje);
    }

    // Pequeño delay para evitar Watchdog Trigger si no hay actividad
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// ==========================================
// 5. TAREA CORE 1: APLICACIÓN (SENSOR + UI)
// ==========================================
void TareaSensor_Code(void * parameter) {
  String estadoSensor = "ON";
  int contador_global = 0;
  DatosCola datosParaEnviar;

  for(;;) {
    // 1. LECTURA SENSOR (Simulado)
    estadoSensor = (estadoSensor == "ON") ? "OFF" : "ON";
    
    // 2. ENVIAR A COLA (TX)
    datosParaEnviar.payload = estadoSensor;
    datosParaEnviar.id_mensaje = contador_global;
    
    // Enviamos a la cola para que el Core 0 lo procese
    xQueueSend(xCola_TX, &datosParaEnviar, portMAX_DELAY);
    contador_global++;

    // 3. ACTUALIZAR PANTALLA
    // Esta sección lee las variables compartidas de RX protegidas por Mutex
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "Core 1: TX ID " + String(contador_global));
    
    // -- SECCIÓN CRÍTICA (Lectura RX) --
    if (xSemaphoreTake(xMutex_RX, (TickType_t)100) == pdTRUE) {
      if (rx_nuevo_dato) {
        Heltec.display->drawString(0, 20, "RX de: 0x" + String(rx_remite, HEX));
        Heltec.display->drawString(0, 30, "Msg: " + rx_mensaje);
        Heltec.display->drawString(0, 40, "RSSI: " + String(rx_rssi));
      } else {
        Heltec.display->drawString(0, 20, "Esperando datos...");
      }
      xSemaphoreGive(xMutex_RX);
    }
    Heltec.display->display();

    // 4. CSMA/CA (JITTER ALEATORIO)
    // Aquí implementamos la lógica de "espera aleatoria" del primer código.
    // 6000ms base + 0 a 3000ms aleatorios.
    long tiempo_wait = 6000 + random(3000);
    vTaskDelay(tiempo_wait / portTICK_PERIOD_MS);
  }
}

// ==========================================
// 6. LÓGICA DE RECEPCIÓN DETALLADA (HELPER)
// ==========================================
void procesar_recepcion(int packetSize) {
  if (packetSize == 0) return;

  // Variables temporales locales (no compartidas aún)
  String paq_rcb = "";
  byte   d_envio = LoRa.read();
  byte   d_remite = LoRa.read();
  byte   id_msg = LoRa.read();
  byte   len_msg = LoRa.read();

  // Leer Payload
  while (LoRa.available()) {
    paq_rcb += (char)LoRa.read();
  }

  // --- FILTROS DE INTEGRIDAD ---
  
  // 1. Checksum de longitud
  if (paq_rcb.length() != len_msg) {
    Serial.println("[RX Error] Longitud corrupta");
    return; 
  }

  // 2. Filtro de Dirección
  if (d_envio != dir_local && d_envio != 0xFF) {
    Serial.println("[RX Ignorado] No es para mi");
    return;
  }

  // --- GUARDADO SEGURO (MUTEX) ---
  // Si pasa los filtros, actualizamos las variables globales para la pantalla
  if (xSemaphoreTake(xMutex_RX, (TickType_t)10) == pdTRUE) {
    rx_mensaje = paq_rcb;
    rx_remite  = d_remite;
    rx_rssi    = LoRa.packetRssi();
    rx_nuevo_dato = true;
    xSemaphoreGive(xMutex_RX); // Liberar llave
    
    // Feedback visual RX
    digitalWrite(LED_PIN, HIGH); vTaskDelay(50); digitalWrite(LED_PIN, LOW);
    Serial.print("[Core 0] RX de 0x"); Serial.print(d_remite, HEX);
    Serial.print(": "); Serial.println(paq_rcb);
  }
}
