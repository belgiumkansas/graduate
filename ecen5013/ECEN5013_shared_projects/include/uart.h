/**********************************************************
* Name: uart.h
*
* Date: 10/1/16
*
* Description: This module defines the UART interface
*
* Author: Ben Heberlein
*
***********************************************************/

#ifndef UART_H
#define UART_H

#include <stdint.h>

uint8_t init_uart();

uint8_t tx_buf();

uint8_t tx_char(uint8_t ch);

uint8_t tx_string(uint8_t *str, int32_t length);

int8_t rx_valid();

uint8_t rx_char();

#endif
