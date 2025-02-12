// irda.cpp
#include "irda.h"
#include <cstring> // Nécessaire pour strlen
#include <stdio.h> // Utile pour les fonctions d'entrée/sortie standard
#include <driver/uart.h> // Pour les fonctions UART
#include <string.h>

#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024

// Initialize the IRDA communication
void irda_init(int txPin, int rxPin) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

// Send a packet to the IRDA
void irda_send_packet(const char* data) {
    uart_write_bytes(UART_NUM, data, strlen(data));
}

// Receive a packet from the IRDA
void irda_receive_packet(char* buffer, int length) {
    int len = uart_read_bytes(UART_NUM, (uint8_t*)buffer, length, 100 / portTICK_RATE_MS);
    if (len > 0) {
        buffer[len] = '\0'; // Terminate the string
    } else {
        buffer[0] = '\0';
    }
}

// Flush the UART buffer
void irda_flush() {
    uart_flush(UART_NUM);
}