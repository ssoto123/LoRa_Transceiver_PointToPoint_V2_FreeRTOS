# 📡 LoRa P2P Multitasking con FreeRTOS (Dual Core)

![Platform](https://img.shields.io/badge/Hardware-Heltec_V2-blue)
![OS](https://img.shields.io/badge/OS-FreeRTOS-green)
![Protocol](https://img.shields.io/badge/Protocol-LoRa_P2P-orange)
![Status](https://img.shields.io/badge/Status-Educational-success)

Este repositorio contiene la implementación avanzada de un sistema de comunicación LoRa Punto a Punto para la asignatura **Sistemas de Sensores** (Maestría en IoT).

A diferencia de la programación secuencial clásica de Arduino (`setup` + `loop`), este proyecto utiliza **FreeRTOS** para explotar la arquitectura **Dual Core** del ESP32, separando la lógica de comunicación crítica (Radio) de la lógica de aplicación (Sensores/UI).

---

## 🧠 Arquitectura del Sistema (RTOS)

El objetivo es simular el comportamiento de dispositivos profesionales (como los nodos LoRaWAN), donde el stack de comunicaciones no debe ser bloqueado por la lectura lenta de un sensor.

### Distribución de Núcleos (Cores)

| Núcleo | Tarea Asignada | Prioridad | Descripción |
| :--- | :--- | :--- | :--- |
| **Core 0** | `TareaLoRa_Code` | **Alta (2)** | **Gestor de Radio.** Se encarga exclusivamente de mantener la radio escuchando (RX) y transmitir (TX) cuando se le solicita. Simula un módem dedicado. |
| **Core 1** | `TareaSensor_Code` | **Normal (1)** | **Aplicación de Usuario.** Lee sensores, gestiona la pantalla OLED y decide cuándo generar datos. |

### Mecanismos de Comunicación Inter-Procesos (IPC)

Como tenemos dos "cerebros" trabajando en paralelo, necesitamos formas seguras de pasar información entre ellos sin que choquen:

1.  **📬 La Cola (Queue) - `xCola_TX`**:
    * Funciona como un **Buzón**.
    * El **Core 1** (Sensor) deposita ahí los datos que quiere enviar y sigue trabajando.
    * El **Core 0** (Radio) revisa el buzón; si hay cartas, las envía.
    * *Beneficio:* El sensor no se queda "congelado" esperando a que la radio termine de transmitir.

2.  **🔑 El Semáforo (Mutex) - `xMutex_RX`**:
    * Funciona como una **Llave única**.
    * Protege las variables donde se guardan los mensajes recibidos (`rx_mensaje`).
    * Evita el error de **Condición de Carrera**: Impide que el Core 1 lea un mensaje incompleto mientras el Core 0 lo está escribiendo.

---

## 🔄 Comparativa: Secuencial vs. RTOS

Aquí explicamos cómo se adaptó la lógica original (Secuencial) a este nuevo paradigma:

| Característica | Versión 1 (Secuencial) | Versión 2 (FreeRTOS) |
| :--- | :--- | :--- |
| **Estructura** | Un solo bucle `loop()` infinito. | Múltiples Tareas (`Tasks`) independientes. |
| **Espera** | `delay()` o `millis()` (Bloqueante/Polling). | `vTaskDelay()` (Libera la CPU para otra tarea). |
| **Recepción** | Si el sensor tarda, la radio **no escucha**. | La radio escucha el 100% del tiempo (Core 0). |
| **CSMA/CA** | Lógica lineal compleja en el loop. | Dividido: **Jitter** en Core 1, **Carrier Sense** en Core 0. |

---

## ⚙️ Configuración y Parámetros

### 1. Direccionamiento (Capa de Enlace)
Para establecer comunicación, configura los IDs en el encabezado del código:

```cpp
// Nodo A
byte dir_local   = 0xC1; 
byte dir_destino = 0xD3; 

// Nodo B
byte dir_local   = 0xD3; 
byte dir_destino = 0xC1;
