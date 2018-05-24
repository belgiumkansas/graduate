/**********************************************************
* Name:
*
* Date:
*
* Description:
* (fill in detailed description of the module here)
*
* Author: Jeff Venicx
*
**********************************************************/
#include<stdint.h>
#include<stdbool.h>
#include<stdio.h>
#define DEBUG

void log_char( char *data, int32_t length, bool newline){
  #ifdef DEBUG
  //print char array
  for(int i = 0; i < length; i++){
    printf("%c", data[i]);
  }
  //add newline
  if(newline) printf("\n");
  #endif
}

void log_uint8_t(uint8_t *data , int32_t length, bool newline){
  #ifdef DEBUG
  //print uint8_t array
  for(int i = 0; i < length; i++){
    printf("%u", data[i]);
  }
  //add newline
  if(newline)printf("\n");
  #endif
}

void log_hexdump(uint8_t *data, int32_t length, bool newline){
  #ifdef DEBUG
  //print uint8_t aray
  for(int i = 0; i < length; i++){
    printf("%02x ", data[i]);
    //newline every 8 hex's
    if(!((i+1)%8))printf("\n");
  }
  //add newline
  if(newline)printf("\n");
  #endif
}
