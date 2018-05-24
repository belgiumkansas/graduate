/**********************************************************
* Name: cmds.h
*
* Date: 10/31/2016
*
* Description: This module contains interfaces for the 
* generic command interface
*
* Author: Ben Heberlein
*
***********************************************************/

#include <stdint.h>

MAX_DATA_SIZE 65535

typdef enum cmd_e {
	LED_TOGGLE = 0x01,
	LED_RED = 0x02,
	LED_GRN = 0x03,
	LED_BLUE = 0x024
} cmd_t;

typdef struct cmd_msg_s {
	cmd_t command;
	uint8_t length;
	uint8_t data[MAX_DATA_SIZE];
	uint16_t checksum;
} cmd_msg_t;

uint16_t checksum_calc (cmd_msg_t *msg);
