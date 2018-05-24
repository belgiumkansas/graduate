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

#include "osc.h"
#include "log.h"

#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <fcntl.h>
#include <endian.h>
#include <stdbool.h>
#include <string.h>

#define UINT8_BY8_TO_UINT64(x) (x[7] | (x[6] << 8) | (x[5] << 16) | (x[4] << 24) | (x[3] << 32) | (x[2] << 40) | (x[1] << 48)| (x[0] << 56))
#define UINT8_BY4_TO_UINT32(x)  x[3] | (x[2] << 8) | (x[1] << 16) | (x[0] << 24)
//#define NULL 0x00

int osc_find_bundle(int fd){
  uint8_t bundle[8] = {0};
  char* bundle_check = "#bundle\0";
  bool is_bundle = false;
  uint8_t bundle_flag = 0;

  //TODO complicated fix this shiz
  while(bundle_flag < 8){
    //read byte by byte check for '#'
    while((bundle[0] != '#')) read(fd, &bundle,1);

    //read next 7 bytes after #
    read(fd, &bundle[1], 7);

    //reset bundle flag
    bundle_flag = 0;
    //check if complete bunde
    for(int i = 0; i<8; i++){
      //if correct increment bundle flag
      if ((bundle[i] == bundle_check[i])){
        bundle_flag++;
      }
    }
    if(bundle_flag == 8);
    else log_char("bad bundle",10, true);
  }

  //print bundle buffer for check
  log_char(&bundle[0], 7, false);
  log_hexdump(&bundle[7], 1, false);

  return 1;
}

int osc_write_timetag(int fd, osc_bundle * bundle){
  //TODO add error
  read(fd, &bundle->buffer[0], 8);
  //set buffer head marker
  bundle->marker = &bundle->buffer[8];
  log_char(" timetag ",9 ,false);
  return 1;
}

int osc_get_next_message(int fd, osc_bundle* bundle, osc_message* message){

  //record size of message into buffer
  read(fd, &bundle->marker[0], 4);
  //log message size
  log_uint8_t(&bundle->marker[0], 4, false);
  uint32_t message_size = UINT8_BY4_TO_UINT32(bundle->marker);

  //record begining at message address
  message->buffer = &bundle->marker[0];
  message->len = message_size;
  //increment buffer marker
  bundle->marker = &bundle->marker[4];
  //set message marker and buffer start
  message->marker = bundle->marker;
  //read message
  read(fd, &bundle->marker[0], message_size);
  //data dump of data
  //log_char(&bundle->marker[0], message_size, true);
  //log_hexdump(&bundle->marker[0], message_size, true);
  //increment bundle marker
  bundle->marker = &bundle->marker[message_size];

  //assume address max 32
  uint8_t i = 0;
  char address[16] = {0};
  //record addresse and inrement marker
  while((message->marker[i] != '\0') & (i < 16)){
    address[i] = message->marker[i];
    i++;
  }
  address[i] = '\0';
  //log_uint8_t(&i, 1, true);
  //log_char(address, i, true);

  if(strcmp(address, "/quaternion") == 0){
    log_char("/quaternion", 11 ,false);
  }
  else if(strcmp(address, "/humidity") == 0){
    log_char("/humidity", 9 ,false);
  }
  else{
    log_char("error", 5, true);
    return 0;
  }
  //increment the marker
  message->marker = &message->marker[i];
  //find start of
  i = 0;
  while((message->marker[i] != ',') & (i < 4)){
    i++;
  }
  //if comma marker found continute
  if(message->marker[i] == ',')i++;
  else return 0;
  //increment message marker
  message->marker = &message->marker[i];
  //set as message address
  message->address = message->marker;
  //count messages



  //go through messages
  i = 0;
  while((message->address[i] != '\0') & (i < 6)){
    if(message->address[i] == 'f'){
      //float osc_parse_next_float(message, )
    }
    i++;
  }

  printf("\n");

  return 1;
}



float osc_get_next_float(osc_message *message, uint8_t *marker){

 return 0;
}



int get_message(osc_message* message, int fd){

  char buffer[256];

  uint8_t size[4] = {0};

  read(fd,&size, 4);

  printf("size array: ");
  for(int i = 0; i <4; i++){
    printf("%02x ", size[i]);
  }
  printf("\n");

  //shift bytes into uint32_t
  uint32_t size2 = size[3] | (size[2] << 8) | (size[1] << 16) | (size[0] << 24);

  printf("size = %d\n", size2);

  uint8_t data_buffer[size2+1];

  read(fd, &data_buffer, size2+1);

  printf("dump data buffer ascii\n");
  for(int i = 0; i < size2+1; i++){
    printf("(%02X),(%C);", data_buffer[i], data_buffer[i]);
  }
  printf("\n");

  //printf("size = %d", size);

  //TODO add in time tags
  //uint64_t timetag;
  //read(fd, &timetag, 8);
  //printf("timetag = %u\n", timetag);


  /*

  //osc message structure
  osc_message a;
  //TODO junky fix this
  enum complete osc_addresse = INCOMPLETE;

  char buff[256];
  int j = 0;
  //read message address
  while(osc_addresse == INCOMPLETE){
    read(fd, &buff[j], 1);

    if(buff[j] == ','){
      osc_addresse = COMPLETE;
      break;
    }
    j ++;
  }

  printf("osc addresse name: ");
  for(int i = 0; i < j; i++){
    printf("%c", buff[i]);
  }
  printf("\n");




  char buff_msgs[16];

  int test = 1;
  int k = 0;
  int data_count = 0;
  //message types
  while(test){

    read(fd, &buff_msgs[k], 1);
    if(buff_msgs[k] != 'f'){
      test = 0;
      break;
    }
    if(buff_msgs[k] == 'f'){
      data_count += 4;
    }
    k++;
  }

  printf("messages types; ");
  for(int i = 0; i < k; i++){
    printf("%c, ", buff_msgs[i]);
  }
  printf("\n");


  printf("data count = %d\n", data_count);
  char data_buff[256] = {0};

  read(fd,&data_buff, data_count+10);

  printf("dump buffer contents:\n");
  for(int i = 0 ; i < data_count; i++){
    printf("%02x, ", data_buff[i]);
  }
  printf("\n");

  printf("tail data: ");
  for( int i = 0; i < 10; i++){
    printf("%c\n",data_buff[data_count + i] );

  }

  printf("\n");




    //while(file_type != ','){}
*/
  return 1;
}

int write_message(int fd){
  char bundle_array[8] = "#bundle";
  uint8_t bundle_2 [32] = {0};

  for( int i = 0; i < 7; i++){
    bundle_2[i] = (uint8_t)bundle_array[i];
  }



  //empty time tag
  for (int i = 0; i < 8; i++){
    bundle_2[i+8] = 0;
  }

  //size tag
  for( int i = 0; i < 3; i++){
    bundle_2[i+16] = 0;
  }
  bundle_2[19] = 12;

  char addresse[9] = "/identify";

  for( int i = 0; i < 9; i++){
    bundle_2[i+20] = (uint8_t)addresse[i];
  }
  printf("test bundle conversion: \n");
  for( int i = 0; i < 32; i++){
    printf("(%C, %02X)",bundle_2[i], bundle_2[i]);
    if(((i+1)%4 == 0) & (i != 0)){printf("\n");}
  }
  printf("\n");

  write(fd,bundle_2, 32);

  uint8_t bundle_3[12] = {0};
  for( int i = 0 ; i < 9; i++){
    bundle_3[i] = (uint8_t)addresse[i];
  }

  write(fd, bundle_3, 12);

}
