#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// ===================== CONFIGURAZIONE =====================

// Intervallo di acquisizione di default (ms)
#define ACQUISITION_PERIOD_MS_DEFAULT 500
// Numero massimo di valori in coda tra acquisizione e processing
#define TEMP_QUEUE_LENGTH 10
// Numero di letture da mantenere nel buffer per la media mobile
#define AVG_BUFFER_SIZE 10

// ===================== HANDLE GLOBALI =====================

static QueueHandle_t xTempQueue = NULL;         // Coda per passare temperature
static SemaphoreHandle_t xSerialMutex = NULL;   // Mutex per proteggere la stampa
static SemaphoreHandle_t xThresholdMutex = NULL;// Mutex per proteggere la soglia
static SemaphoreHandle_t xAvgBufferMutex = NULL;// Mutex per proteggere buffer media
static SemaphoreHandle_t xPeriodMutex = NULL;   // Mutex per proteggere intervallo
static SemaphoreHandle_t xRunMutex = NULL;      // Mutex per proteggere stato running

// ===================== VARIABILI GLOBALI =====================

// Soglia temperatura per lampeggio LED
static float currentThreshold = 33.0f;
// Intervallo di acquisizione (ms)
static int acquisitionPeriodMs = ACQUISITION_PERIOD_MS_DEFAULT;
// Stato di esecuzione: false = fermo, true = sta acquisendo
static bool isRunning = false;

// Buffer circolare per memorizzare le ultime letture
static float avgBuffer[AVG_BUFFER_SIZE] = {0};
static int avgIndex = 0;
static int avgCount = 0;

// ===================== FUNZIONI DI SUPPORTO =====================

/**
 * @brief Stampa thread-safe su seriale.
 * Usa un mutex per evitare che più task scrivano contemporaneamente.
 */
static void safe_print(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    xSemaphoreTake(xSerialMutex, portMAX_DELAY);
    vprintf(fmt, args);
    xSemaphoreGive(xSerialMutex);
    va_end(args);
}

/**
 * @brief Legge la temperatura interna del RP2040.
 */
static float read_onboard_temperature(void) {
    adc_select_input(4); // canale 4 = sensore integrato
    uint16_t raw = adc_read();
    const float conv = 3.3f / (1 << 12);
    float voltage = raw * conv;
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}

// Funzioni thread-safe per soglia
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

// Funzioni thread-safe per intervallo acquisizione
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

// Funzioni thread-safe per stato running
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

/**
 * @brief Reset buffer media mobile.
 */
static void clearAvgBuffer(void) {
    xSemaphoreTake(xAvgBufferMutex, portMAX_DELAY);
    memset(avgBuffer, 0, sizeof(avgBuffer));
    avgIndex = 0;
    avgCount = 0;
    xSemaphoreGive(xAvgBufferMutex);
}

/**
 * @brief Stampa la legenda dei comandi disponibili.
 */
static void printMenuHelp(void) {
    safe_print("\n[MENU] Comandi disponibili:\n");
    safe_print("  r = avvia rilevazione (reset)\n");
    safe_print("  t = sospendi rilevazione\n");
    safe_print("  p = mostra soglia attuale\n");
    safe_print("  s<val> = imposta nuova soglia (es: s30)\n");
    safe_print("  m = media ultime 10 letture\n");
    safe_print("  i<ms> = imposta intervallo acquisizione (es: i1000)\n");
    safe_print("  g = mostra intervallo acquisizione attuale\n\n");
}

// ===================== TASK 1: ACQUISIZIONE =====================
/**
 * @brief Legge periodicamente la temperatura e la inserisce in coda
 * solo se lo stato "isRunning" è true.
 */
static void vTaskAcquisition(void *pvParameters) {
    (void) pvParameters;
    for (;;) {
        if (getIsRunning()) {
            float temp = read_onboard_temperature();

            // Invia in coda
            xQueueSend(xTempQueue, &temp, portMAX_DELAY);

            // Aggiorna buffer media
            xSemaphoreTake(xAvgBufferMutex, portMAX_DELAY);
            avgBuffer[avgIndex] = temp;
            avgIndex = (avgIndex + 1) % AVG_BUFFER_SIZE;
            if (avgCount < AVG_BUFFER_SIZE) avgCount++;
            xSemaphoreGive(xAvgBufferMutex);
        }
        // Attendi il periodo corrente
        vTaskDelay(pdMS_TO_TICKS(getAcquisitionPeriod()));
    }
}

// ===================== TASK 2: ELABORAZIONE =====================
/**
 * @brief Legge dalla coda e stampa la temperatura istantanea.
 * Controlla la soglia e lampeggia il LED se necessario.
 */
static void vTaskProcessing(void *pvParameters) {
    (void) pvParameters;
    for (;;) {
        float newTemp;
        if (xQueueReceive(xTempQueue, &newTemp, portMAX_DELAY) == pdTRUE) {
            if (getIsRunning()) {
                if (newTemp > getThreshold()) {
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                safe_print("[TEMP] %.2f°C\n", newTemp);
            }
        }
    }
}

// ===================== TASK 3: MENU UTENTE =====================
/**
 * @brief Gestisce i comandi dell'utente ricevuti su seriale.
 */
static void vTaskMenu(void *pvParameters) {
    (void) pvParameters;
    char inputBuf[16];
    int idx = 0;

    // Stampa legenda comandi (dopo aver atteso connessione USB in main)
    printMenuHelp();

    for (;;) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT && c != EOF) {
            if (c == '\r' || c == '\n') {
                inputBuf[idx] = '\0';
                if (idx > 0) {
                    char cmd = inputBuf[0];
                    switch (cmd) {
                        case 'r': { // start
                            clearAvgBuffer(); // reset finestra
                            setIsRunning(true);
                            safe_print("[MENU] Rilevazione avviata.\n");
                            break;
                        }
                        case 't': { // stop
                            setIsRunning(false);
                            safe_print("[MENU] Rilevazione sospesa.\n");
                            printMenuHelp();
                            break;
                        }
                        case 'p': {
                            safe_print("[MENU] Soglia attuale: %.2f°C\n", getThreshold());
                            break;
                        }
                        case 's': {
                            float val = atof(&inputBuf[1]);
                            if (val > 0.0f) {
                                setThreshold(val);
                                safe_print("[MENU] Nuova soglia impostata: %.2f°C\n", val);
                            } else {
                                safe_print("[MENU] Valore soglia non valido.\n");
                            }
                            break;
                        }
                        case 'm': {
                            xSemaphoreTake(xAvgBufferMutex, portMAX_DELAY);
                            if (avgCount == 0) {
                                safe_print("[MENU] Nessuna lettura disponibile.\n");
                            } else {
                                float sum = 0;
                                for (int i = 0; i < avgCount; i++) sum += avgBuffer[i];
                                float avg = sum / avgCount;
                                safe_print("[MENU] Media ultime %d letture: %.2f°C\n", avgCount, avg);
                            }
                            xSemaphoreGive(xAvgBufferMutex);
                            break;
                        }
                        case 'i': {
                            int ms = atoi(&inputBuf[1]);
                            if (ms >= 100 && ms <= 5000) {
                                setAcquisitionPeriod(ms);
                                safe_print("[MENU] Nuovo intervallo acquisizione: %d ms\n", ms);
                            } else {
                                safe_print("[MENU] Valore intervallo non valido (100-5000 ms).\n");
                            }
                            break;
                        }
                        case 'g': {
                            safe_print("[MENU] Intervallo acquisizione: %d ms\n", getAcquisitionPeriod());
                            break;
                        }
                        default:
                            safe_print("[MENU] Comando sconosciuto: %s\n", inputBuf);
                            break;
                    }
                }
                idx = 0;
            } else {
                if (idx < (int)(sizeof(inputBuf) - 1)) {
                    inputBuf[idx++] = (char)c;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ===================== MAIN =====================
int main() {
    // Inizializza periferiche base
    stdio_init_all();

    // ATTENDI che la USB sia pronta, così la legenda non si perde
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    // Configura ADC per sensore temperatura
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    // Configura LED integrato
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

    // Crea coda e mutex
    xTempQueue = xQueueCreate(TEMP_QUEUE_LENGTH, sizeof(float));
    xSerialMutex = xSemaphoreCreateMutex();
    xThresholdMutex = xSemaphoreCreateMutex();
    xAvgBufferMutex = xSemaphoreCreateMutex();
    xPeriodMutex = xSemaphoreCreateMutex();
    xRunMutex = xSemaphoreCreateMutex();

    if (!xTempQueue || !xSerialMutex || !xThresholdMutex ||
        !xAvgBufferMutex || !xPeriodMutex || !xRunMutex) {
        printf("[FATAL] Errore creazione risorse!\n");
        while (1);
    }

    // Crea i task
    xTaskCreate(vTaskAcquisition, "Acquisition", 512, NULL, 2, NULL);
    xTaskCreate(vTaskProcessing,  "Processing",  512, NULL, 3, NULL);
    xTaskCreate(vTaskMenu,        "Menu",        1024, NULL, 1, NULL);

    // Avvia scheduler FreeRTOS
    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }
}
