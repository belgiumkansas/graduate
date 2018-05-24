/**********************************************************
* Name: memory.c
*
* Date: 9/14/16
*
* Description: This module contains several memory
* manipulation functions.
*
* Author: Ben Heberlein & Jeff Venicx
*
***********************************************************/

#include <stdint.h>
#include "memory.h"

#ifdef FRDM
#include "dma.h"

/*dma_memmove : int8_t dma_memmove(uint8_t *src, uint8_t *dst, uint32_t length);
*   returns   : 1 for succes -1 failure
*   src       : source pointer for move
*   dst       : destination pointer for move
*   length    : length of data to be moved
*Created by   : Ben Heberlein
*Date         : 10-29/16
*Description  : move memory from one location to another using DMA
****************************************************************************/
int8_t dma_memmove(uint8_t *src, uint8_t *dst, uint32_t length) {
    //invalid strings
    if(!src|!dst) return -1;
    //if memmove to same location
    if(src == dst) return 0;

    uint32_t diff = 0;
    uint32_t overlap = 0;

    // Check if dst is higher in memory and if there is overlap
    if (dst > src) {
    	diff = dst - src;
    	// if there is overlap, transfer overlapped section in reverse
    	if (diff < length) {
    		overlap = length - diff;
    		for(int i = 0; i < overlap; i++) {
    			dma_start(src+length-i-1, dst+length-i-1, 1);
    		}

    		// Transfer the rest normally
    		dma_start(src, dst, length-overlap);
    	} else {
    		// Transfer normally for no overlap
    		dma_start(src, dst, length);
    	}

    // If dst < src, the DMA will work fine and transmit from low to high
    } else {
    	dma_start(src, dst, length);
    }

    //proper execution
    return 0;
}

/****************************************************************************
*dma_memzero : int8_t dma_memzero(uint8_t *src, uint32_t length);
*   returns  : 1 for succes -1 failure
*   src      : start point of func
*   length   : length of data to be used
*Created by  : Ben Heberlein
*Date        : 10-29-16
*Description : place zeros in memory starting at source for length
****************************************************************************/
int8_t dma_memzero(uint8_t *src, uint32_t length) {
    //invalid string
    if(!src) return -1;

    uint32_t src_mod = 0;
    if (((uint32_t) src) % 4 != 0) {
    	src_mod = 4 - ((uint32_t) src) % 4;
    }
    uint32_t len_mod = (length - src_mod) % 4;
    if (src_mod != 0) {
    	dma_zero(src, 1, src_mod);
    }
	dma_zero(src+src_mod, 4, length - len_mod - src_mod);
	if (len_mod != 0) {
		dma_zero(src+length-len_mod, 1, len_mod);
	}

    return 0;
}

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
int8_t my_memmove(uint8_t *src, uint8_t *dst, uint32_t length) {
    //invalid strings
    if(!src|!dst) return -1;
    //if memmove to same location
    if(src == dst) return 0;

    //if src < dest start at end to prevent possible overlap/overwrite
    if(src < dst){
      //start from end
      src = src + length-1;
      dst = dst + length-1;
      for (uint32_t i = 0; i < length; i++){
          *dst = *src;
          dst--;
          src--;
      }
    }

    //if dest < src start at begining to prevent overlap
    else if(src > dst) {
      for(uint32_t i = 0; i < length; i++){
        *dst = *src;
        dst++;
        src++;
      }
    }
    //unknown error
    else return -1;
    //proper execution
    return 0;
}

/****************************************************************************
*my_memzero  : int8_t my_memzero(uint8_t *src, uint32_t length);
*   returns  : 1 for succes -1 failure
*   src      : start point of func
*   length   : length of data to be used
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : place zeros in memory starting at source for length
****************************************************************************/
int8_t my_memzero(uint8_t *src, uint32_t length) {
    //invalid string
    if(!src) return -1;

    //place zeros
    for(uint32_t i = 0; i < length; i++){
      *src = 0;
      src++;
    }

    return 0;
}

/****************************************************************************
*my_reverse  : int8_t my_reverse(uint8_t *src, uint32_t length);
*   returns  : 1 for succes -1 failure
*   src      : start point of func
*   length   : length of data to be used
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : put data in the reverse order
****************************************************************************/
int8_t my_reverse(uint8_t *src, uint32_t length) {
    //invalid string
    if(!src) return -1;
     //invalid length
     if(length < 1) return -1;

     //end of string pointer
     uint8_t *end = src + length -1;
     //swap space variable
     uint8_t swap;
    //perform swap function
     while(src < end){
       swap = *src;
       *src = *end;
       *end = swap;
       src++;
       end--;
     }

    return 0;
}
