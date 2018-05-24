/**********************************************************
* Name: memory.h
*
* Date: 9/14/16
*
* Description: This module contains several memory
* manipulation functions.
*
* Author: Ben Heberlein & Jeff Venicx
*
***********************************************************/

#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>

#ifdef FRDM
/****************************************************************************
*dma_memmove  : int8_t dma_memmove(uint8_t *src, uint8_t *dst, uint32_t length);
*   returns   : 1 for succes -1 failure
*   src       : source pointer for move
*   dst       : destination pointer for move
*   length    : length of data to be moved
*Created by   : Ben Heberlein
*Date         : 10/29/16
*Description  : move memory from one location to another with DMA
****************************************************************************/
int8_t dma_memmove(uint8_t *src, uint8_t *dst, uint32_t length);

/****************************************************************************
*dma_memzero : int8_t dma_memzero(uint8_t *src, uint32_t length);
*   returns  : 1 for succes -1 failure
*   src      : start point of func
*   length   : length of data to be used
*Created by  : Ben Heberlein
*Date        : 10-29-16
*Description : place zeros in memory starting at source for length
****************************************************************************/
int8_t dma_memzero(uint8_t *src, uint32_t length);

#endif

/****************************************************************************
*my_memmove  : int8_t my_memmove(uint8_t *src, uint8_t *dst, uint32_t length);
*   returns  : 1 for succes -1 failure
*   src      : source pointer for move
*   dst      : destination pointer for move
*   length   : length of data to be moved
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : move memory from one location to another
****************************************************************************/
int8_t my_memmove(uint8_t *src, uint8_t *dst, uint32_t length);

/****************************************************************************
*my_memzero  : int8_t my_memzero(uint8_t *src, uint32_t length);
*   returns  : 1 for succes -1 failure
*   src      : start point of func
*   length   : length of data to be used
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : place zeros in memory starting at source for length
****************************************************************************/
int8_t my_memzero(uint8_t *src, uint32_t length);

/****************************************************************************
*my_reverse  : int8_t my_reverse(uint8_t *src, uint32_t length);
*   returns  : 1 for succes -1 failure
*   src      : start point of func
*   length   : length of data to be used
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : put data in the reverse order
****************************************************************************/
int8_t my_reverse(uint8_t *src, uint32_t length);

#endif
