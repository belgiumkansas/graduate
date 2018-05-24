/**********************************************************
* Name: XIO.c
*
* Date: 11/21/2016
*
* Description:
* Implenentation of XIO interface
*
* Author: Jeff Venicx
*
**********************************************************/

#include "XIO.h"
#include "osc.h"
#include "tinyosc.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <endian.h>





int init_XIO(){


  printf("hello world init_XIO\n");
  //TODO don't hardcode device find a better way
  int fd = open("/dev/ttyACM0", O_RDWR);

    /*
    printf("charr array = \n");
    for( int i = 0; i < 1000; i++){
        printf("%c", byte[i]);
    }
      printf("\n end array \n");
*/

}


int read_XIO(int fd){

  osc_bundle bundle;
  //assumes no bundle greater than 256 bytes
  uint8_t buffer[256] = {0};
  //attach buffer to osc_bundle
  bundle.buffer = &buffer[0];
  // will read fd untill head of buffer found
  int error = osc_find_bundle(fd);
  //error in finding bundle
  if(!error) return 0;
  //get time tag next
  osc_write_timetag(fd, &bundle);
  //create array of pointers to osc_messages
  osc_message messages[5];
  bool bundle_end = false;
  int message_count = 0;
  //loop to read a bundle's messages
  while(!bundle_end & (message_count < 1)){
    osc_get_next_message(fd, &bundle, &messages[message_count]);
    message_count++;
  }



  //get_message(&test, fd);

  return 1;
}



int write_XIO(int fd){
  tosc_bundle bundle;
  tosc_message message;





  return 1;
}





//end spacer
