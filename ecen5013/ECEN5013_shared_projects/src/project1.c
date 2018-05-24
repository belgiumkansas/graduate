/**********************************************************
* Name: project1.c
*
* Date: 9/14/16
*
* Description: This module contains the main routine for
* ECEN 5013 project 1
*
* Author: Ben Heberlein & Jeff Venicx
*
**********************************************************/
#include "project1.h"
#include "memory.h"
#include "data.h"
#include <stdio.h>

void project_1_report() {
    //initialize report array and pointers
    uint8_t array[32];
    uint8_t *aptr_1 = array;
    uint8_t *aptr_2 = &array[8];
    uint8_t *aptr_3 = &array[16];


    //aptr_1: Initialize 16 bytes starting at
    //this aptr_1 to the numbers from 31-46.
    //Do not modify the pointer address
    uint8_t i;
    uint8_t value = 31;
    for (i = 0; i < 16; i++){
        aptr_1[i] = value;
        value++;
    }

    //aptr_3: Initialize the contents from this pointer
    //to the end of the array to zeros using memzero.
    //Do not modify the pointer address
    my_memzero(aptr_3, 16);

    //Use memmoveto move 8 bytes from aptr_1 to aptr_3
    my_memmove(aptr_1, aptr_3, 8);

    //Use memmove to move 16 bytes from aptr_2to aptr_1.
    my_memmove(aptr_2, aptr_1, 16);

    //Use printf to print out the entire 32 byte array in a nicely formatted way.
    for(i = 0; i < 32; i++ ){
      printf("%3u ",array[i]);
      if(!((i+1)%8)) printf("\n");
    }
    printf("\n");
}
