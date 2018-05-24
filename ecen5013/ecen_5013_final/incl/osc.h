/**********************************************************
* Name:
*
* Date:
*
* Description:
* (description of the module here)
*
* Author:
*
**********************************************************/
#ifndef _OSC_H_
#define _OSC_H_
#include<stdint.h>

/**********************************************************
*function    : void function(void arguments);
*   returns  : nothing it is void
*   arguments: the input arguments
*Created by  : Jeff Venicx
*Date        :
*Description :
*Notes       :
**********************************************************/
enum osc_address {QUATERNIAN, HUMIDITY};


typedef struct osc_message {
  char *format;       // a pointer to the format field
  uint8_t *marker;    // the current read head
  uint8_t *buffer;    // the original message data (also points to the address)
  uint8_t *address;   // location of address in buffer
  uint8_t *type;      // start location of types
  uint32_t len;       // length of the buffer data
} osc_message;

typedef struct osc_bundle {
  uint8_t *marker;       // the current write head (where the next message will be written)
  uint8_t *buffer;    // the original buffer
  uint32_t bufLen;    // the byte length of the original buffer
  uint32_t bundleLen; // the byte length of the total bundle
} osc_bundle;



int osc_find_bundle(int fd);

int osc_write_timetag(int fd, osc_bundle *buffer);

int osc_get_next_message(int fd, osc_bundle* bundle, osc_message* message);

float osc_parse_float(osc_message *message, uint8_t *marker);

int get_message(osc_message* message, int fd);

int write_message(int fd);

#endif
