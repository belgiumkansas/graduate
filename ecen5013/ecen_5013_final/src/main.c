/**********************************************************
* Name: main.c
*
* Date: 11/21/2016
*
* Description:
* main executalble
*
* Author: Jeff Venicx
*
**********************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "XIO.h"
#define DEBUG

int main(int argc, const char* argv[]){

  printf("hello world \n");

  int fd = init_XIO();

  int rd_error = read_XIO(fd);

  //int rw_error = write_XIO(fd);

  printf("rd error=  %d\n", rd_error);
  return 0;
}
