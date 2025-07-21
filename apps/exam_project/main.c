#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// ==================== CONFIGURAZIONI ====================

// Tempo tra un'acquisizione e la successiva (ms)
#define ACQUISITION_PERIOD_MS 500
// Soglia temperatura per far lampeggiare il LED
#define TEMP_THRESHOLD_C 33.0f
// Lunghezza coda per passare le temperature
#define TEMP_QUEUE_LENGTH 10

// ==================== HANDLE GLOBALI ====================
static QueueHandle_t xTempQueue = NULL;       // coda per dati temperatura
static SemaphoreHandle_t xSerialMutex = NULL; // mutex per proteggere la seriale

// ==================== FUNZIONE DI LETTURA ====================
/**
 * @brief Legge la temperatura interna dal sensore integrato nel RP2040.
 * @return valore della temperatura in gradi Celsius.
 */
float read_onboard_temperature() {
    adc_select_input(4); // Canale 4: sensore interno
    uint16_t raw = adc_read();
    const float conv = 3.3f / (1 << 12);
    float voltage = raw * conv;
    return 27.0f - (voltage - 0.706f) / 0.001721f;
}

// ==================== TASK 1: ACQUISIZIONE ====================
/**
 * @brief Task che legge periodicamente la temperatura e la inserisce nella coda.
 */
void vTaskAcquisition(void *pvParameters) {
    (void) pvParameters;
    float temp;
    for (;;) {
        temp = read_onboard_temperature();
        xQueueSend(xTempQueue, &temp, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(ACQUISITION_PERIOD_MS));
    }
}

// ==================== TASK 2: ELABORAZIONE ====================
/**
 * @brief Task che riceve le temperature dalla coda, controlla la soglia
 *        e gestisce il LED, poi invia i dati in output (stampa).
 */
void vTaskProcessing(void *pvParameters) {
    (void) pvParameters;
    float receivedTemp;
    for (;;) {
        // Attende dato dalla coda
        if (xQueueReceive(xTempQueue, &receivedTemp, portMAX_DELAY) == pdTRUE) {
            // Controllo soglia
            if (receivedTemp > TEMP_THRESHOLD_C) {
                // Lampeggio LED
                gpio_put(PICO_DEFAULT_LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_put(PICO_DEFAULT_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            // Proteggi la stampa con mutex
            xSemaphoreTake(xSerialMutex, portMAX_DELAY);
            printf("[PROCESS] Temp: %.2f °C\n", receivedTemp);
            xSemaphoreGive(xSerialMutex);
        }
    }
}

// ==================== TASK 3: OUTPUT (opzionale separato) ====================
/**
 * @brief Task placeholder per eventuale output separato.
 * In questo esempio, la stampa è già fatta nel task Processing.
 */
void vTaskOutput(void *pvParameters) {
    (void) pvParameters;
    for (;;) {
        // Se volessi separare la stampa, potresti ricevere qui
        // dati elaborati da una seconda coda.
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== MAIN ====================
int main() {
    // Inizializza stdio per la seriale USB
    stdio_init_all();

    // Inizializza ADC per lettura sensore interno
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    // Configura LED integrato
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

    // Crea coda e mutex
    xTempQueue = xQueueCreate(TEMP_QUEUE_LENGTH, sizeof(float));
    xSerialMutex = xSemaphoreCreateMutex();

    // Crea i task con priorità differenziate
    xTaskCreate(vTaskAcquisition, "Acquisition", 512, NULL, 2, NULL);
    xTaskCreate(vTaskProcessing,  "Processing",  512, NULL, 3, NULL);
    xTaskCreate(vTaskOutput,      "Output",      512, NULL, 1, NULL);

    // Avvia lo scheduler FreeRTOS
    vTaskStartScheduler();

    // Non dovrebbe mai arrivare qui
    while (1) {
        tight_loop_contents();
    }
}
// ==================== FINE CODICE ====================
