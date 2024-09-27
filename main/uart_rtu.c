#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "uart_rtu.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE 256

static QueueHandle_t uart_queue;
static SemaphoreHandle_t rx_sem;

// Send data
int send_data(uint8_t *buf, int len) {
    return uart_write_bytes(UART_NUM, (const char*)buf, len);
}

// Receive data
int receive_data(uint8_t *buf, int bufsz, int timeout, int bytes_timeout) {
    int len = 0;
    int rc;
    TickType_t start = xTaskGetTickCount();

    while (1) {
        if (xSemaphoreTake(rx_sem, pdMS_TO_TICKS(timeout)) == pdTRUE) {
            rc = uart_read_bytes(UART_NUM, buf + len, bufsz, pdMS_TO_TICKS(bytes_timeout));
            if (rc > 0) {
                len += rc;
                bufsz -= rc;
                if (bufsz == 0)
                    break;
            } else if (rc == 0) {
                break;
            }
        } else {
            break;
        }

        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS >= timeout)
            break;
    }

    return len;
}

// UART event handling task
static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    // Handle received data
                    xSemaphoreGive(rx_sem);
                    break;
                // Handle other UART events...
                default:
                    break;
            }
        }
    }
}

// UART initialization
int uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));

    rx_sem = xSemaphoreCreateBinary();
    if (rx_sem == NULL) {
        ESP_LOGE("UART", "Failed to create rx semaphore");
        return -1;
    }

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    return ESP_OK;
}