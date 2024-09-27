set(PROJECT_NAME "MODBUS")

idf_component_register(SRCS "main.c" "tcp_slave.c" "agile_modbus_rtu.c" "agile_modbus_tcp.c" "agile_modbus.c" "uart_rtu.c"
                        INCLUDE_DIRS ".")
