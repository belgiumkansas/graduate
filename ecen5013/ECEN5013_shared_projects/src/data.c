/**********************************************************
* Name: data.c
*
* Date: 9/14/16
*
* Description: This module contains several data
* manipulation functions.
*
* Author: Ben Heberlein & Jeff Venicx
*
***********************************************************/

#include "stdint.h"
#include "data.h"
#include "memory.h"
#include <string.h>
#include <stdio.h>

/****************************************************************************
*my_itoa     : int my_itoa(int8_t *str, int32_t data, int32_t base);
*   returns  : length of ASCII array
*   str      : array pointer for transformed data
*   data     : number to be transformed
*   base     : base number value
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : takes a interger and converts it into ASCII string
*Notes       : Only works with base 10 or lower
****************************************************************************/
int my_itoa(uint8_t *str, int32_t data, int32_t base) {

    //functional iterator variable
    int i = 0;
    //sign flag
    int8_t sign = 1;
    //check for negative number
    if (data < 0){
      sign = -1;
      data = data *-1;
    }
    //add digits to ASCII string
    while(data != 0){
        str[i] = (uint8_t) (data % base + '0');
        if (str[i] >= 10 + '0') {
             str[i] = str[i] + 7;
        }
        i++;
        data = data/base;
    }
    //add the negative sign to array
    if(sign < 1){
      str[i] = '-';
      i++;
    }
    //add null terminator
    str[i] = '\0';
    //reverse the string

    my_reverse(str, i);
    return i;
}

/****************************************************************************
*my_atoi     : int32_t my_atoi(int8_t *str);
*   returns  : number value
*   str      : string pointer of ASCII
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : takes in ASCII string and converts it too interger
****************************************************************************/
int32_t my_atoi(int8_t *str) {
    //result storage variable
    int32_t result = 0;
    //sign flag
    int8_t sign = 1;
    //check for negative value
    if(*str == '-'){
      sign = -1;
      str++;
    }
    //convert ASCII to value
    while(*str != '\0'){
      //check ASCII value is a number
      if((*str < '0') | (*str > '9')) return 0;
      //
      result = result * 10;
      result = result + *str -'0';
      str++;
      //check if value rolled over
      //aka it was to large for the data type
      if(result < 0) return 0;
    }
    //return result multiplied by sign
    return result*sign;
}

/****************************************************************************
*dump_memory : void dump_memory(uint8_t *start, uint32_t length);
*   returns  : void
*   start    : starting address
*   length   : length of data
*Created by  : Jeff Venicx
*Date        : 9-16-16
*Description : printf memory of length starting at address start
****************************************************************************/
void dump_memory(uint8_t *start, uint32_t length) {

    if(!start) printf("invalid start addresse\n");

    if(length < 1) printf("invalid length < 1\n");
    //iterate through memory and dump
    for(uint32_t i = 0; i < length; i++){
        //new line after 11 bytes
        if(!(i%11) & i != 0) printf("\n");
        printf("%02X ", *start);
        start++;
    }
    printf("\n");
}

/****************************************************************************
*big_to_little: uint32_t big_to_little(uint32_t data);
*   returns   : transformed data
*   data      : date to be transformed
*Created by   : Jeff Venicx
*Date         : 9-16-16
*Description  : convert from big to little endian
****************************************************************************/
uint32_t big_to_little(uint32_t data) {
    //the value to be returned
    uint32_t return_val;
    //shift and mask data
    return_val = ((data>>24)&0xff)|
                 ((data<<8)&0xff0000)|
                 ((data>>8)&0xff00)|
                ((data<<24)&0xff000000);
    return return_val;
}

/****************************************************************************
*little_to_big: uint32_t big_to_little(uint32_t data);
*   returns   : transformed data
*   data      : date to be transformed
*Created by   : Jeff Venicx
*Date         : 9-16-16
*Description  : convert from little to big endian
****************************************************************************/
uint32_t little_to_big(uint32_t data) {
    //endianness change is the same in either direction
    return big_to_little( data);
}

/****************************************************************************
*ftoa         : void ftoa(float value, uint8_t *ascii);
*   returns   : void
*   value     : float value to be transformed to ascii
*   ascii     : array with ascii values of value
*Created by   : Jeff Venicx
*Date         : 10-5-16
*Description  : convert a float into a array of ascii
****************************************************************************/

void my_ftoa(float value, uint8_t *ascii){
  //extract interger
  int interger = (int)value;
  //printf("interger part: %d\n", interger);

  //extract float portion
  int multiplier = 1;
  if(interger < 0){
    multiplier = -1;
  }
  float fraction = value - (float)interger;
  fraction = fraction * 1000 * multiplier;

  my_itoa(ascii, interger, 10);
  int i = strlen(ascii);

  ascii[i] = 46;
  i++;
  fraction = (int)fraction;
  my_itoa(&ascii[i],fraction, 10);

}

void concat_strings(uint8_t *out_buf, uint8_t *a, uint8_t *b, uint8_t a_size, uint8_t b_size) {
	uint8_t i = 0;
	for (i = 0; i < a_size; i++) {
		*(out_buf+i) = *(a+i);
	}
	for (i = 0; i < b_size; i++) {
		*(out_buf+a_size+i) = *(b+i);
	}
}
