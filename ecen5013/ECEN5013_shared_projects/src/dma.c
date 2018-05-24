/**********************************************************
* Name: dma.c
*
* Date: 10/25/2016
*
* Description: This module contains implementations of DMA
* control functions
*
* Author: Ben Heberlein
*
***********************************************************/

#include "dma.h"
#include <stdint.h>
#include "MKL25Z4.h"
#include "log.h"

volatile uint8_t ready_flag;
const uint32_t zero_word = 0;

/* Initialization function for DMA control */
int8_t dma_init() {

	ready_flag = 1;

	// Set clock gate to DMA module
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	// Enable interrupt
	DMA_DCR0 |= DMA_DCR_EINT_MASK;

	NVIC_EnableIRQ(DMA0_IRQn);
}


/* Start a DMA transfer */
int8_t dma_start(uint8_t *src, uint8_t *dst, uint32_t len) {

	if (len >= 1<<24) {
		Log0("DMA transfer too large.", 23);
		return -1;
	}

	// Spin on other transfers
	while(!ready_flag){
		__NOP;
	}
	
	// Set transfer flag
	ready_flag = 0;
	
	DMA_SAR0 = (uint32_t) src;
	DMA_DAR0 = (uint32_t) dst;
	DMA_DSR_BCR0 &= ~DMA_DSR_BCR_BCR_MASK;
	DMA_DSR_BCR0 |= (len & DMA_DSR_BCR_BCR_MASK);

	// Check for valid configuration
	if (DMA_DSR_BCR0 & DMA_DSR_BCR_CE_MASK) {
		Log0("Invalid DMA configuration. Transfer aborted.", 44);
		DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;
		return -2;
	}

	// Auto alignment mode optimizes byte sizes for us
	DMA_DCR0 |= DMA_DCR_AA_MASK;

	// Transfer sizes, we want src > dst for auto alignment mode
	DMA_DCR0 &= ~DMA_DCR_SSIZE_MASK;
	DMA_DCR0 &= ~DMA_DCR_DSIZE_MASK;
	DMA_DCR0 |= DMA_DCR_DSIZE(1);

	// Source and destination increment
	DMA_DCR0 |= DMA_DCR_SINC_MASK;
	DMA_DCR0 |= DMA_DCR_DINC_MASK;


	DMA_DCR0 |= DMA_DCR_START_MASK;

}

int8_t dma_zero(uint8_t *dst, uint8_t chunk_size, uint32_t len) {

	if (!(chunk_size == 4 || chunk_size == 2 || chunk_size == 1)) {
		Log0("Invalid DMA chunk size.", 23);
		return -1;
	}

	if (len >= 1<<24) {
		Log0("DMA transfer too large.", 23);
		return -2;
	}

	// Spin on other transfers
	while(!ready_flag){
		__NOP;
	}
	
	// Set transfer flag
	ready_flag = 0;


	// Can't use auto align for zero
	DMA_DCR0 &= ~DMA_DCR_AA_MASK;

	DMA_SAR0 = (uint32_t) (&zero_word);
	DMA_DAR0 = (uint32_t) dst;
	DMA_DSR_BCR0 &= ~DMA_DSR_BCR_BCR_MASK;
	DMA_DSR_BCR0 |= (len & DMA_DSR_BCR_BCR_MASK);

	// Check for valid configuration
	if (DMA_DSR_BCR0 & DMA_DSR_BCR_CE_MASK) {
		Log0("Invalid DMA configuration. Transfer aborted.", 44);
		DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;
		return -3;
	}

	DMA_DCR0 &= ~DMA_DCR_SSIZE_MASK;
	DMA_DCR0 &= ~DMA_DCR_DSIZE_MASK;
	if (chunk_size == 2) {
		DMA_DCR0 |= DMA_DCR_SSIZE(2);
		DMA_DCR0 |= DMA_DCR_DSIZE(2);
	} else if (chunk_size == 1) {
		DMA_DCR0 |= DMA_DCR_SSIZE(1);
		DMA_DCR0 |= DMA_DCR_DSIZE(1);
	}

	// Only desination increment
	DMA_DCR0 &= ~DMA_DCR_SINC_MASK;
	DMA_DCR0 |= DMA_DCR_DINC_MASK;


	DMA_DCR0 |= DMA_DCR_START_MASK;

}

void DMA0_IRQHandler(void) {
	if (DMA_DSR_BCR0 & DMA_DSR_BCR_CE_MASK) {
		Log0("Invalid DMA transfer. Data may be incomplete.", 45);
	}

	// Clear DONE and CE flags, set flag for new transfer
	DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;
	ready_flag = 1;
}
