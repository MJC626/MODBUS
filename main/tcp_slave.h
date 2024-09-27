#ifndef TCP_SLAVE_H
#define TCP_SLAVE_H

#include "esp_err.h"
#include "mbcontroller.h"

#ifdef __cplusplus
extern "C" {
#endif

// Define any necessary constants
#define MB_TCP_PORT_NUMBER      (CONFIG_FMB_TCP_PORT_DEFAULT)
#define MB_SLAVE_ADDR           (CONFIG_MB_SLAVE_ADDR)

// Function declarations
esp_err_t init_services(void);
esp_err_t destroy_services(void);
esp_err_t slave_init(mb_communication_info_t* comm_info);
esp_err_t slave_destroy(void);
void slave_operation_func(void *arg);

#ifdef __cplusplus
}
#endif

#endif // TCP_SLAVE_H