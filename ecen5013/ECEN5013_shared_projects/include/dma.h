/**********************************************************
 * * Name: dma.h
 * *
 * * Date: 10/26/2016
 * *
 * * Author: Ben Heberlein
 * *
 * * Description: This file contains function declarations for
 * * DMA control functions.
 * *
 * **********************************************************/

#ifndef DMA_H
#define DMA_H

#include <stdint.h>

int8_t dma_init();
int8_t dma_start(uint8_t *, uint8_t *, uint32_t);
int8_t dma_zero(uint8_t *dst, uint8_t chunk_size, uint32_t len);
#endif

