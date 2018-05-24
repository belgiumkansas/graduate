
/**********************************************************
* Name: profiler.h
*
* Date: 10/7/2016
*
* Description: This module contains definitions for the
* profiler
*
* Author: Ben Heberlein & Jeff Venicx
*
***********************************************************/

// header file for profiler

void delay(int milliseconds);

void profile_memory(uint32_t length);

void profile_data ();

void profile_malloc();

void profile_circbuf();

void profile_log();

void profile_init();

int profile_convert_time(struct timeval *result, struct timeval *x, struct timeval *y);

void profile_all();
