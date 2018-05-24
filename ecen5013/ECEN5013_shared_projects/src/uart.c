/**********************************************************
* Name: uart.c
*
* Date: 10/1/2016
*
* Description: This module contains implementations for
* UART functions
*
* Author: Ben Heberlein
*
***********************************************************/
#include "MKL25Z4.h"
#include <stdint.h>
#include <stdlib.h>
#include "log.h"
#include "timer.h"
#include "circbuf.h"

#define BAUD_RATE 	57600
#define OVERSAMPLE 	16
#define CB_BUFFER_CAP 512

circbuf_t *tx_cb;
circbuf_t *rx_cb;

uint8_t init_uart() {
	// Enable port A
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	// Enable UART Rx on pin PTA1
	PORTA_PCR1 = PORT_PCR_MUX(0x2);
	// Enable UART Tx on pin PTA2
	PORTA_PCR2 = PORT_PCR_MUX(0x2);
	// Set MCGFLLCLK as UART0 clock source
	SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1);
	// Open the UART0 clock gate
	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;

	// Find division to put in Baud registers
	uint16_t div = SystemCoreClock / OVERSAMPLE / BAUD_RATE;
	// Set oversampling, should be constant
    UART0_C4 = UARTLP_C4_OSR(OVERSAMPLE-1);
    // Set Baud registers
    UART0_BDH = (div >> 8) & UARTLP_BDH_SBR_MASK;
    UART0_BDL = div & UARTLP_BDL_SBR_MASK;

    // Enable Rx and Tx
    UART0_C2 |= UART0_C2_RE_MASK | UART0_C2_TE_MASK;

	// Enable Rx interrupt
    UART0_C2 |= UART0_C2_RIE_MASK;
    NVIC_EnableIRQ(UART0_IRQn);

    // Initialize Rx and Tx circular buffers
    tx_cb = circbuf_initialize(CB_BUFFER_CAP);
    rx_cb = circbuf_initialize(CB_BUFFER_CAP);
}

uint8_t tx_buf() {
	// There is room in the tx register
	// while(!(UART0_S1_REG(UART0_BASE_PTR) & UART0_S1_TDRE_MASK)){};
	if((UART0_S1_REG(UART0_BASE_PTR) & UART0_S1_TDRE_MASK)) {
		// Check if there is tx data
		if (!circbuf_buffer_empty(tx_cb)) {
			// Write the character to the data register
			UART0_D_REG(UART0_BASE_PTR) = circbuf_remove_item(tx_cb);
		}
	}
}

uint8_t tx_char(uint8_t ch) {
	// Add ch to buffer
	circbuf_add_item(ch, tx_cb);
	tx_buf();
}

uint8_t tx_string(uint8_t *str, int32_t length) {
	while(length > 0) {
		length--;
		tx_char(*str++);
	}
}

// Wrapper function to see if there is valid Rx data
int8_t rx_valid() {
	return (circbuf_buffer_empty(rx_cb) == 0);
}

// Wrapper function to read byte
uint8_t rx_char() {
	if (circbuf_buffer_empty(rx_cb)) {
		return 0;
	} else {
		return circbuf_remove_item(rx_cb);
	}
}

void UART0_IRQHandler (void) {
	// Check for Rx character
	if (UART0_S1 & UART_S1_RDRF_MASK) {
		circbuf_add_item(UART0_D, rx_cb);
	}
}
