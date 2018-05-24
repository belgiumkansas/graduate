/**********************************************************
* Name: circbuf.c
*
* Date: 09/25/2016
*
* Author: Ben Heberlein
*
* Description: This file implements several functions to
* manage a circular_buffer.
*
**********************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "circbuf.h"

#define MAX_CAP 1024

/***********************************************************
* circbuf_buffer_full : uint8_t circbuf_buffer_full(circbuf_t *circular_buffer);
*   returns           : 1 for full, 0 for not full, -1 for error
    circular_buffer   : The circular buffer to be checked
* Author              : Ben Heberlein
* Date                : 09/25/2016
* Description         : Check if circular buffer is full
************************************************************/
int8_t circbuf_buffer_full(circbuf_t *circular_buffer) {
    // Check if valid buffer
    if (circular_buffer == NULL) {
        return -1;
    }

    if (circular_buffer->STATUS == FULL) {
        return 1;
    } else {
        return 0;
    }
}

/***********************************************************
* circbuf_buffer_empty : uint8_t circbuf_buffer_empty(circbuf_t *circular_buffer);
*   returns            : 1 for empty, 0 for not empty, -1 for error
*   circular_buffer    : The circular buffer to be checked
* Author               : Ben Heberlein
* Date                 : 09/25/2016
* Description          : Check if circular buffer is empty
***********************************************************/
int8_t circbuf_buffer_empty(circbuf_t *circular_buffer) {
    // Check if valid buffer
    if (circular_buffer == NULL) {
        return -1;
    }

    if (circular_buffer->STATUS == EMPTY) {
        return 1;
    } else {
        return 0;
    }
}

/***********************************************************
* circbuf_add_item  : uint8_t circbuf_add_item(uint8_t data, circbuf_t *circular_buffer);
*   returns         : 0 for success, -1 for failure
*   data            : The data to be added
*   circular_buffer : The circular buffer to be added to
* Author            : Ben Heberlein
* Date              : 09/25/2016
* Description       : Add an item to the circular buffer
***********************************************************/
int8_t circbuf_add_item(uint8_t data, circbuf_t *circular_buffer) {
    // Check if valid buffer
    if (circular_buffer == NULL) {
        return -1;
    }

    // Check if full
    if (circular_buffer->STATUS == FULL) {
        return -1;
    }

    // Set data
    *(circular_buffer->head) = data;

    // Increment head and check for wrap
    circular_buffer->head++;
    if ((circular_buffer->head - circular_buffer->buf) >= circular_buffer->capacity) {
        circular_buffer->head -= circular_buffer->capacity;
    }
    circular_buffer->size++;

    // Set new state
    if (circular_buffer->size == circular_buffer->capacity ||
        circular_buffer->head == circular_buffer->tail) {
        circular_buffer->STATUS = FULL;
    } else {
        circular_buffer->STATUS = PARTIAL;
    }

    return 0;
}

/***********************************************************
* circbuf_remove_item : uint8_t circbuf_remove_item(circbuf_t *circular_buffer);
*   returns           : The data if successful, 0 if buffer is empty
*   circular_buffer   : The circular buffer to get data from
* Author              : Ben Heberlein
* Date                : 09/25/2016
* Description         : Remove an item from the circular buffer
***********************************************************/
uint8_t circbuf_remove_item(circbuf_t *circular_buffer) {
    // Check if valid buffer
    if (circular_buffer == NULL) {
        return 0;
    }

    // Check if empty
    if (circular_buffer->STATUS == EMPTY) {
        return 0;
    }

    // Get data
    uint8_t ret = *(circular_buffer->tail);

    // Increment tail and check for wrap
    circular_buffer->tail++;
    if ((circular_buffer->tail - circular_buffer->buf) >= circular_buffer->capacity) {
        circular_buffer->tail -= circular_buffer->capacity;
    }
    circular_buffer->size--;

    // Set new state
    if (circular_buffer->size == 0) {// || circular_buffer->head == circular_buffer->tail) {
        circular_buffer->STATUS = EMPTY;
    } else {
        circular_buffer->STATUS = PARTIAL;
    }

    return ret;
}

/***********************************************************
* circbuf_initialize : circbuf_t *circbuf_initialize(uint16_t capacity);
*   returns          : A circular buffer with given capacity if successful
*                      or NULL pointer if failure
*   capacity		 : Capacity of the buffer
* Author             : Ben Heberlein
* Date               : 10/5/2016
* Description        : Initialize a new circular buffer
***********************************************************/
circbuf_t *circbuf_initialize(uint16_t capacity) {

  //check zero case
  if(capacity == 0)return NULL;

  // check maximum
  if (capacity > MAX_CAP) return NULL;

  circbuf_t *init = NULL;
	init = (circbuf_t *) malloc(sizeof(circbuf_t));
	if (init == NULL) {
		return NULL;
	}

	init->buf = NULL;
	init->buf = (uint8_t *) malloc(capacity);
	if (init->buf == NULL) {
		free(init);
		return NULL;
	}

	init->head = init->buf;
	init->tail = init->buf;
	init->capacity = capacity;
	init->size = 0;
	init->STATUS = EMPTY;

	return init;
}

/***********************************************************
* circbuf_destroy    : uint8_t circbuf_destroy(circbuf_t *circular_buf);
*   returns          : 0 for successful destroy or -1 for failure
*   circular_buf     : Circular buffer to destroy
* Author             : Ben Heberlein
* Date               : 10/5/2016
* Description        : Destroy an existing circular buffer
***********************************************************/
int8_t circbuf_destroy(circbuf_t *circular_buf) {
	if (circular_buf == NULL) {
		return -1;
	}

	if (circular_buf->buf == NULL) {
		free(circular_buf);
		return -1;
	}

	free(circular_buf->buf);
	free(circular_buf);
	return 0;
}
