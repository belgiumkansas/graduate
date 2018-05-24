/**********************************************************
* Name: data.h
*
* Date: 9/14/16
*
* Description: This module contains several data
* manipulation functions.
*
* Author: Ben Heberlein & Jeff Venicx
*
**********************************************************/

#ifndef DATA_H
#define DATA_H

#include "stdint.h"

/****************************************************************************
*my_itoa     : int my_itoa(uint8_t *str, int32_t data, int32_t base);
*   returns  : length of ASCII array
*   str      : array pointer for transformed data
*   data     : number to be transformed
*   base     : base number value
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : takes a interger and converts it into ASCII string
*Notes       : Only works with base 10 or lower
****************************************************************************/
int my_itoa(uint8_t *str, int32_t data, int32_t base);

/****************************************************************************
*my_atoi     : int32_t my_atoi(int8_t *str);
*   returns  : number value
*   str      : string pointer of ASCII
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : takes in ASCII string and converts it too interger
****************************************************************************/
int32_t my_atoi(int8_t *str);

/****************************************************************************
*dump_memory : void dump_memory(uint8_t *start, uint32_t length);
*   returns  : void
*   start    : starting address
*   length   : length of data
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : printf memory of length starting at address start
****************************************************************************/
void dump_memory(uint8_t *start, uint32_t length);

/****************************************************************************
*big_to_little: uint32_t big_to_little(uint32_t data);
*   returns   : transformed data
*   data      : date to be transformed
*Created by   : Jeff Venicx
*Date         : 9-16-16
*Description  : convert from big to little endian
****************************************************************************/
uint32_t big_to_little(uint32_t data);

/****************************************************************************
*little_to_big: uint32_t big_to_little(uint32_t data);
*   returns   : transformed data
*   data      : date to be transformed
*Created by   : Jeff Venicx
*Date         : 9-16-16
*Description  : convert from little to big endian
****************************************************************************/
uint32_t little_to_big(uint32_t data);

/****************************************************************************
* my_ftoa     : void my_ftoa(float value, uint8_t *ascii);
*   returns   : void
*   value     : float value to be transformed to ascii
*   ascii     : array with ascii values of value
*Created by   : Jeff Venicx
*Date         : 10-5-16
*Description  : convert a float into a array of ascii
****************************************************************************/

void my_ftoa(float value, uint8_t *ascii);

void concat_strings(uint8_t *, uint8_t *, uint8_t *, uint8_t, uint8_t);

#endif
