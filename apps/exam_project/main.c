#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>              // per calcolare la deviazione standard
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// ===================== CONFIGURAZIONE =====================

// Intervallo di acquisizione iniziale (ms)
#define ACQUISITION_PERIOD_MS_DEFAULT 1000
// Lunghezza massima della coda fra acquisizione e processing
#define TEMP_QUEUE_LENGTH 10
// Numero di letture da memorizzare nel buffer circolare per statistiche
#define AVG_BUFFER_SIZE 10

// ===================== TIPI E DEFINIZIONI =====================

// Struct per dati da passare alla coda
typedef struct {
    TickType_t timestamp; // timestamp in tick FreeRTOS
    float temperature; // temperatura in gradi Celsius
} TempSample_t;

// ===================== HANDLE GLOBALI =====================

// Strutture di sincronizzazione e comunicazione
static QueueHandle_t xTempQueue = NULL;         // Coda per passare temperature
static SemaphoreHandle_t xSerialMutex = NULL;   // Mutex per proteggere la seriale
static SemaphoreHandle_t xThresholdMutex = NULL; // Mutex per proteggere la soglia
static SemaphoreHandle_t xAvgBufferMutex = NULL; // Mutex per buffer statistiche
static SemaphoreHandle_t xPeriodMutex = NULL;   // Mutex per intervallo acquisizione
static SemaphoreHandle_t xRunMutex = NULL;      // Mutex per stato running

// Handle dei task, utile per diagnostica stack
static TaskHandle_t hTaskAcq = NULL;
static TaskHandle_t hTaskProc = NULL;
static TaskHandle_t hTaskMenu = NULL;
static TaskHandle_t hTaskStats = NULL;
static TaskHandle_t hTaskDiag = NULL;

// ===================== VARIABILI GLOBALI =====================

// Parametri runtime
static float currentThreshold = 33.0f;          // Soglia per LED
static int acquisitionPeriodMs = ACQUISITION_PERIOD_MS_DEFAULT;
static bool isRunning = false;                  // Stato della rilevazione

// Buffer per statistiche
static float avgBuffer[AVG_BUFFER_SIZE] = {0}; // Buffer circolare per ultime letture
static int avgIndex = 0; // Indice corrente nel buffer cirocolare
static int avgCount = 0; // Numero di elementi validi nel buffer circolare

// ===================== FUNZIONI DI SUPPORTO =====================

/**
 * Stampa thread-safe su seriale.
 * Usa un mutex per evitare interferenze fra task.
 */
static void safe_print(const char *fmt, ...) { //fmt è il formato della stringa
    va_list args; // per gestire argomenti variabili
    va_start(args, fmt); // inizializza la lista di argomenti
    // Prende il mutex per proteggere l'accesso alla seriale
    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
    vprintf(fmt, args);
    xSemaphoreGive(xSerialMutex);
    va_end(args); // termina l'uso della lista di argomenti
}

/**
 * Legge la temperatura dal sensore interno del RP2040.
 */
static float read_onboard_temperature(void) {
    adc_select_input(4); // canale 4 = sensore temperatura interno
    uint16_t raw = adc_read(); // legge il valore grezzo (12-bit)
    const float conv = 3.3f / (1 << 12); // converte da 12-bit a volt
    float voltage = raw * conv;
    return 27.0f - (voltage - 0.706f) / 0.001721f; // formula di conversione
}

// --- Funzioni thread-safe per parametri globali ---

// Soglia
static float getThreshold(void) {
    xSemaphoreTake(xThresholdMutex, portMAX_DELAY);
    float v = currentThreshold;
    xSemaphoreGive(xThresholdMutex);
    return v;
}
static void setThreshold(float v) {
    xSemaphoreTake(xThresholdMutex, portMAX_DELAY);
    currentThreshold = v;
    xSemaphoreGive(xThresholdMutex);
}

// Intervallo
static int getAcquisitionPeriod(void) {
    xSemaphoreTake(xPeriodMutex, portMAX_DELAY);
    int v = acquisitionPeriodMs;
    xSemaphoreGive(xPeriodMutex);
    return v;
}
static void setAcquisitionPeriod(int v) {
    xSemaphoreTake(xPeriodMutex, portMAX_DELAY);
    acquisitionPeriodMs = v;
    xSemaphoreGive(xPeriodMutex);
}

// Stato running
static bool getIsRunning(void) {
    xSemaphoreTake(xRunMutex, portMAX_DELAY);
    bool v = isRunning;
    xSemaphoreGive(xRunMutex);
    return v;
}
static void setIsRunning(bool v) {
    xSemaphoreTake(xRunMutex, portMAX_DELAY);
    isRunning = v;
    xSemaphoreGive(xRunMutex);
}

// Reset del buffer statistico
static void clearAvgBuffer(void) {
    xSemaphoreTake(xAvgBufferMutex, portMAX_DELAY);
    memset(avgBuffer, 0, sizeof(avgBuffer)); // azzera il buffer
    avgIndex = 0;
    avgCount = 0;
    xSemaphoreGive(xAvgBufferMutex);
}

// Stampa legenda menu
static void printMenuHelp(void) {
    safe_print("\n[MENU] Comandi disponibili:\n");
    safe_print("  r = avvia rilevazione (reset buffer)\n");
    safe_print("  t = sospendi rilevazione\n");
    safe_print("  p = mostra soglia attuale\n");
    safe_print("  s<val> = imposta nuova soglia (es: s30)\n");
    safe_print("  m = media ultime 10 letture\n");
    safe_print("  i<ms> = imposta intervallo acquisizione (es: i500)\n");
    safe_print("  g = mostra intervallo acquisizione attuale\n\n");
}

// ===================== TASK 1: ACQUISIZIONE =====================
/**
 * Legge periodicamente la temperatura e la inserisce nella coda.
 * Aggiorna anche il buffer per statistiche.
 */
static void vTaskAcquisition(void *pvParameters) {
    (void) pvParameters; // per evitare warning su parametro non usato
    TempSample_t sample; // inizializza struttura per campioni
    for (;;) {
        if (getIsRunning()) {
            sample.temperature = read_onboard_temperature(); // legge temperatura
            sample.timestamp = xTaskGetTickCount(); // ottiene il timestamp in tick FreeRTOS
            xQueueSend(xTempQueue, &sample, portMAX_DELAY); // invia il campione alla coda
            
            xSemaphoreTake(xAvgBufferMutex, portMAX_DELAY);
            avgBuffer[avgIndex] = sample.temperature;
            avgIndex = (avgIndex + 1) % AVG_BUFFER_SIZE;
            if (avgCount < AVG_BUFFER_SIZE) avgCount++; // incrementa solo se non pieno
            xSemaphoreGive(xAvgBufferMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(getAcquisitionPeriod())); // attende l'intervallo specificato prima di ripetere
    }
}

// ===================== TASK 2: ELABORAZIONE =====================
/**
 * Legge valori dalla coda, li stampa e controlla la soglia.
 */
static void vTaskProcessing(void *pvParameters) {
    (void) pvParameters; // per evitare warning su parametro non usato
    TempSample_t sample;
    for (;;) {
        if (xQueueReceive(xTempQueue, &sample, portMAX_DELAY) == pdTRUE) { // attende un campione dalla coda
            if (getIsRunning()) {
                // Lampeggio LED se sopra soglia
                if (sample.temperature > getThreshold()) {
                    gpio_put(PICO_DEFAULT_LED_PIN, 1); // accende LED
                    vTaskDelay(pdMS_TO_TICKS(100));
                    gpio_put(PICO_DEFAULT_LED_PIN, 0); // spegne LED
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                // Stampa il campione
                safe_print("[TEMP] %lu ms: %.2f°C\n", (unsigned long)sample.timestamp, sample.temperature);
            }
        }
    }
}

// ===================== TASK 3: MENU =====================
/**
 * Legge comandi da seriale e li esegue.
 */
static void vTaskMenu(void *pvParameters) {
    (void) pvParameters; // per evitare warning su parametro non usato
    char inputBuf[16]; // buffer per input comandi
    int idx = 0; // indice corrente nel buffer

    printMenuHelp();

    for (;;) {
        int c = getchar_timeout_us(0); // legge un carattere dalla seriale, senza bloccare
        if (c != PICO_ERROR_TIMEOUT && c != EOF) { // se c'è un carattere valido
            if (c == '\r' || c == '\n') { // premere invio = inviare '\r' o '\n'
                inputBuf[idx] = '\0'; // termina la stringa aggiungendo il terminatore
                if (idx > 0) { // se c'è qualcosa nel buffer
                    char cmd = inputBuf[0];
                    switch (cmd) {
                        case 'r': { clearAvgBuffer(); setIsRunning(true); safe_print("[MENU] Rilevazione avviata.\n"); break; }
                        case 't': { setIsRunning(false); safe_print("[MENU] Rilevazione sospesa.\n"); printMenuHelp(); break; }
                        case 'p': { safe_print("[MENU] Soglia attuale: %.2f°C\n", getThreshold()); break; }
                        case 's': {
                            float val = atof(&inputBuf[1]); // converte il resto della stringa in float
                            if (val > 0.0f) {
                                setThreshold(val);
                                safe_print("[MENU] Nuova soglia: %.2f°C\n", val);
                            } else
                                safe_print("[MENU] Valore soglia non valido.\n");
                            break;
                        }
                        case 'm': {
                            xSemaphoreTake(xAvgBufferMutex, portMAX_DELAY);
                            if (avgCount == 0) safe_print("[MENU] Nessuna lettura disponibile.\n");
                            else {
                                float sum = 0;
                                for (int i = 0; i < avgCount; i++) sum += avgBuffer[i];
                                float avg = sum / avgCount;
                                safe_print("[MENU] Media ultime %d letture: %.2f°C\n", avgCount, avg);
                            }
                            xSemaphoreGive(xAvgBufferMutex);
                            break;
                        }
                        case 'i': {
                            int ms = atoi(&inputBuf[1]); // converte il resto della stringa in intero
                            if (ms >= 500 && ms <= 2000) {
                                setAcquisitionPeriod(ms);
                                safe_print("[MENU] Nuovo intervallo: %d ms\n", ms);
                            } else
                                safe_print("[MENU] Valore intervallo non valido (500-2000 ms).\n");
                            break;
                        }
                        case 'g': { safe_print("[MENU] Intervallo acquisizione: %d ms\n", getAcquisitionPeriod()); break; }
                        default: { safe_print("[MENU] Comando sconosciuto: %s\n", inputBuf); break; }
                    }
                }
                idx = 0; // resetta l'indice per il prossimo comando
            } else {
                if (idx < (int)(sizeof(inputBuf) - 1)) inputBuf[idx++] = (char)c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Attende un po' per non saturare la CPU
    }
}

// ===================== TASK 4: STATISTICHE PERIODICHE =====================
/**
 * Ogni 10 secondi calcola e stampa min, max e deviazione standard
 * delle letture nel buffer, ma solo se la rilevazione è attiva.
 */
static void vTaskStats(void *pvParameters) {
    (void) pvParameters; // per evitare warning su parametro non usato
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // ogni 10 secondi
        if (!getIsRunning()) continue;    // esegue solo se acquisizione attiva

        xSemaphoreTake(xAvgBufferMutex, portMAX_DELAY);
        if (avgCount > 0) {
            // Calcola min e max
            float min = avgBuffer[0], max = avgBuffer[0], sum = 0;
            for (int i = 0; i < avgCount; i++) {
                float v = avgBuffer[i];
                if (v < min) min = v;
                if (v > max) max = v;
                sum += v;
            }
            // Calcola la deviazione standard
            float avg = sum / avgCount;
            float varSum = 0;
            for (int i = 0; i < avgCount; i++) {
                float diff = avgBuffer[i] - avg;
                varSum += diff * diff;
            }
            float stdDev = sqrt(varSum / avgCount);
            safe_print("[STATS] Min: %.2f°C | Max: %.2f°C | Dev.Std: %.2f°C\n", min, max, stdDev);
        } else {
            safe_print("[STATS] Nessuna lettura disponibile.\n");
        }
        xSemaphoreGive(xAvgBufferMutex);
    }
}

// ===================== TASK 5: DIAGNOSTICA =====================
/**
 * Ogni 10 secondi stampa informazioni FreeRTOS:
 * heap libero e stack residuo dei task principali,
 * ma solo se la rilevazione è attiva.
 */
static void vTaskDiagnostics(void *pvParameters) {
    (void) pvParameters; // per evitare warning su parametro non usato
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // ogni 10 secondi
        if (!getIsRunning()) continue;    // esegue solo se acquisizione attiva

        safe_print("[DIAG] Heap libero: %u bytes\n", xPortGetFreeHeapSize());
        // Stack residuo (high water mark) per ciascun task
        safe_print("[DIAG] Stack residuo Acquisition: %u\n", uxTaskGetStackHighWaterMark(hTaskAcq));
        safe_print("[DIAG] Stack residuo Processing: %u\n", uxTaskGetStackHighWaterMark(hTaskProc));
        safe_print("[DIAG] Stack residuo Menu: %u\n", uxTaskGetStackHighWaterMark(hTaskMenu));
        safe_print("[DIAG] Stack residuo Stats: %u\n", uxTaskGetStackHighWaterMark(hTaskStats));
        safe_print("[DIAG] Stack residuo Diagnostics: %u\n", uxTaskGetStackHighWaterMark(hTaskDiag));
    }
}

// ===================== MAIN =====================
int main() {
    stdio_init_all();
    // Attendi che la USB sia pronta prima di stampare
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    // Inizializza ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    // Configura LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

    // Crea strutture di sincronizzazione
    xTempQueue = xQueueCreate(TEMP_QUEUE_LENGTH, sizeof(TempSample_t));
    xSerialMutex = xSemaphoreCreateMutex();
    xThresholdMutex = xSemaphoreCreateMutex();
    xAvgBufferMutex = xSemaphoreCreateMutex();
    xPeriodMutex = xSemaphoreCreateMutex();
    xRunMutex = xSemaphoreCreateMutex();

    if (!xTempQueue || !xSerialMutex || !xThresholdMutex ||
        !xAvgBufferMutex || !xPeriodMutex || !xRunMutex) {
        printf("[FATAL] Errore creazione risorse!\n");
        while (1); // blocca il sistema se non riesce a creare le risorse
    }

    // Crea task principali e salva gli handle per diagnostica
    xTaskCreate(vTaskAcquisition, "Acquisition", 512,  NULL, 2, &hTaskAcq);
    xTaskCreate(vTaskProcessing,  "Processing",  512,  NULL, 3, &hTaskProc);
    xTaskCreate(vTaskMenu,        "Menu",        1024, NULL, 1, &hTaskMenu);
    xTaskCreate(vTaskStats,       "Stats",       512,  NULL, 1, &hTaskStats);
    xTaskCreate(vTaskDiagnostics, "Diagnostics", 512,  NULL, 1, &hTaskDiag);

    // Avvia scheduler
    vTaskStartScheduler();

    // Non dovrebbe mai arrivare qui
    while (1) {
        tight_loop_contents();
    }
}
