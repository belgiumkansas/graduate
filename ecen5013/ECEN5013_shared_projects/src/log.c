/**********************************************************
* Name: log.c
*
* Date: 10/3/2016
*
* Description: This module contains the implementation for
* the UART logger. This will either use UART for the FRDM
* board or printf for native builds and the Beagle Bone.
*
* Author: Ben Heberlein
*
***********************************************************/

#include <stdint.h>
#ifdef FRDM
#include "uart.h"
#else
#include <stdio.h>
#endif
#include <data.h>

void Log0(uint8_t *data, int32_t length) {
#ifdef DEBUG
#ifdef FRDM
	// use UART if we are on the Freedom Freescale board
	tx_string(data, length);
	tx_char('\r');
#else
	// else use normal printf
	for (; length > 0; length--) {
		printf("%c",*data++);
	}
	printf("\n");
#endif
#endif
}

void Log1(uint8_t *data, int32_t length, uint8_t *param, int32_t param_size) {
#ifdef DEBUG
#ifdef FRDM
	// use UART if we are on the Freedom Freescale board
	tx_string(data, length);
	tx_string(" -- sent with payload:", 22);
	while (param_size > 0) {
		int32_t to_print = *param++;
		uint8_t src[10] = "0000000000";

		tx_char(' ');
		tx_string("0x",2);
		my_itoa(src, to_print, 16);
		tx_string(src, 2);

		param_size--;
	}
	tx_char('\r');
#else
	// else use normal printf
	while (length > 0) {
		length--;
		printf("%c",*data++);
	}
	printf(" -- sent with payload ");
	while (param_size > 0) {
		int32_t to_print = *param++;
		int8_t *src;
		src = my_itoa(src, to_print, 16);

		printf("0x");
		printf("%c", (*src) >> 8);
		printf("%c", (*src));
		printf(" ");
		param_size--;
	}
	printf("\n");
#endif
#endif
}

void Log3(uint8_t *data, int32_t length) {
#ifdef DEBUG
#ifdef FRDM
	// use UART if we are on the Freedom Freescale board
	tx_string(data, length);

#else
	// else use normal printf
	for (; length > 0; length--) {
		printf("%c",*data++);
	}
#endif
#endif
}
