#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "mbcontroller.h"
#include "modbus_params.h"
#include "agile_modbus.h"
#include "agile_modbus_rtu.h"
#include "uart_rtu.h"
#include "tcp_slave.h"
#include "modbus_params.h" // Include this header for the register structures

#define MODBUS_MASTER_TASK_STACK_SIZE 4096
#define MODBUS_SLAVE_TASK_STACK_SIZE 4096

static const char *TAG = "MODBUS_COMBINED";

// Modbus master buffers
static uint8_t master_send_buf[AGILE_MODBUS_MAX_ADU_LENGTH];
static uint8_t master_recv_buf[AGILE_MODBUS_MAX_ADU_LENGTH];

// Use the existing extern declarations from modbus_params.h
extern discrete_reg_params_t discrete_reg_params;
extern holding_reg_params_t holding_reg_params;
extern coil_reg_params_t coil_reg_params;
extern input_reg_params_t input_reg_params;

// Function to set up initial register data
static void setup_reg_data(void) {
    discrete_reg_params.discrete_input0 = 1;
    discrete_reg_params.discrete_input1 = 0;
    discrete_reg_params.discrete_input2 = 1;
    discrete_reg_params.discrete_input3 = 0;
    discrete_reg_params.discrete_input4 = 1;
    discrete_reg_params.discrete_input5 = 0;
    discrete_reg_params.discrete_input6 = 1;
    discrete_reg_params.discrete_input7 = 0;

    holding_reg_params.holding_data0 = 100;
    holding_reg_params.holding_data1 = 200;
    holding_reg_params.holding_data2 = 300;
    holding_reg_params.holding_data3 = 400;
    holding_reg_params.holding_data4 = 500;
    holding_reg_params.holding_data5 = 600;
    holding_reg_params.holding_data6 = 700;
    holding_reg_params.holding_data7 = 800;

    coil_reg_params.coils_port0 = 0x55;
    coil_reg_params.coils_port1 = 0xAA;

    input_reg_params.input_data0 = 1.12;
    input_reg_params.input_data1 = 2.34;
    input_reg_params.input_data2 = 3.56;
    input_reg_params.input_data3 = 4.78;
    input_reg_params.input_data4 = 1.12;
    input_reg_params.input_data5 = 2.34;
    input_reg_params.input_data6 = 3.56;
    input_reg_params.input_data7 = 4.78;
}

// 更新保持寄存器值
static void update_slave_registers(uint16_t* data, size_t len, uint16_t start_address) {
    for (size_t i = 0; i < len; i++) {
        switch (start_address + i) {
            case 0:
                holding_reg_params.holding_data0 = data[i];
                break;
            case 1:
                holding_reg_params.holding_data1 = data[i];
                break;
            case 2:
                holding_reg_params.holding_data2 = data[i];
                break;
            case 3:
                holding_reg_params.holding_data3 = data[i];
                break;
            case 4:
                holding_reg_params.holding_data4 = data[i];
                break;
            case 5:
                holding_reg_params.holding_data5 = data[i];
                break;
            case 6:
                holding_reg_params.holding_data6 = data[i];
                break;
            case 7:
                holding_reg_params.holding_data7 = data[i];
                break;
        }
    }
}

// Modbus master task
static void modbus_master_task(void *pvParameters) {
    agile_modbus_rtu_t ctx_rtu;
    agile_modbus_t *ctx = &ctx_rtu._ctx;
    agile_modbus_rtu_init(&ctx_rtu, master_send_buf, sizeof(master_send_buf), master_recv_buf, sizeof(master_recv_buf));
    agile_modbus_set_slave(ctx, 1);
    
    int request_counter = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
        
        switch (request_counter) {
            case 0:
            {
                // Read holding registers from RTU slave
                int send_len = agile_modbus_serialize_read_registers(ctx, 0, 4);
                send_data(ctx->send_buf, send_len);
                int read_len = receive_data(ctx->read_buf, ctx->read_bufsz, 1000, 20);
                
                if (read_len > 0) {
                    uint16_t reg_values[4];
                    int rc = agile_modbus_deserialize_read_registers(ctx, read_len, reg_values);
                    if (rc >= 0) {
                        ESP_LOGI(TAG, "Master0: Read Holding Registers:");
                        for (int i = 0; i < 4; i++) {
                            ESP_LOGI(TAG, "Register[%d]: 0x%04X", i, reg_values[i]);
                        }
                        // Update TCP slave registers with the read values
                        update_slave_registers(reg_values, 4, 0);
                    } else {
                        ESP_LOGE(TAG, "Master: Read failed. Error code: %d", -128 - rc);
                    }
                } else {
                    ESP_LOGI(TAG, "Master: Read timeout.");
                }
                break;
            }
            case 1:
            {
                // Read input registers from RTU slave
                int send_len = agile_modbus_serialize_read_registers(ctx, 4, 4);
                send_data(ctx->send_buf, send_len);
                int read_len = receive_data(ctx->read_buf, ctx->read_bufsz, 1000, 20);
                
                if (read_len > 0) {
                    uint16_t reg_values[4];
                    int rc = agile_modbus_deserialize_read_registers(ctx, read_len, reg_values);
                    if (rc >= 0) {
                        ESP_LOGI(TAG, "Master1: Read Holding Registers:");
                        for (int i = 0; i < 4; i++) {
                            ESP_LOGI(TAG, "Register[%d]: 0x%04X", i, reg_values[i]);
                        }
                        // Update TCP slave input registers with the read values
                        update_slave_registers(reg_values, 4, 4);
                    } else {
                        ESP_LOGE(TAG, "Master: Read failed. Error code: %d", -128 - rc);
                    }
                } else {
                    ESP_LOGI(TAG, "Master: Read timeout.");
                }
                break;
            }
        }
        
        request_counter = (request_counter + 1) % 2;
    }
}


// Modbus slave task
static void modbus_slave_task(void *pvParameters) {
    mb_communication_info_t comm_info = { 0 };
    
#if !CONFIG_EXAMPLE_CONNECT_IPV6
    comm_info.ip_addr_type = MB_IPV4;
#else
    comm_info.ip_addr_type = MB_IPV6;
#endif
    comm_info.ip_mode = MB_MODE_TCP;
    comm_info.ip_port = MB_TCP_PORT_NUMBER;
    
    ESP_ERROR_CHECK(slave_init(&comm_info));
    
    // The Modbus slave logic
    slave_operation_func(NULL);
    
    ESP_ERROR_CHECK(slave_destroy());
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    ESP_ERROR_CHECK(example_connect());

    // Initialize UART for Modbus RTU
    ESP_ERROR_CHECK(uart_init());

    // Set up initial register data
    setup_reg_data();

    // Create Modbus master task
    xTaskCreate(modbus_master_task, "modbus_master_task", MODBUS_MASTER_TASK_STACK_SIZE, NULL, 5, NULL);

    // Create Modbus slave task
    xTaskCreate(modbus_slave_task, "modbus_slave_task", MODBUS_SLAVE_TASK_STACK_SIZE, NULL, 5, NULL);
}