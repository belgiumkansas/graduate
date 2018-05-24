#include "hw1.h"
#include<stdio.h>



char reverse(char * str, int length){

  //invalid string
  if(!str) return 'S';

  //invalid length
  if(length < 2) return 'L';

  //check string length
  int i = 0;
  while(str[i] != '\0') i++;
  //length mismatch error
  if(length != i) return 'M';

  //set variables for reverse swap
  char * end = &str[length-1];
  char * start = str;
  char swap;

  //iterate through string swapping opposite ends
  while(start < end){
    swap = *start;
    *start = *end;
    *end = swap;
    start++;
    end--;
  }

  //successfull reverse
  return '0';
}
