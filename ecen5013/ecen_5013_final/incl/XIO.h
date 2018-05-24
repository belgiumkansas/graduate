/**********************************************************
* Name: XIO.h
*
* Date: 11/21/2016
*
* Description: the hardware interface for XIO IMU
*
* Author: Jeff Venicx
*
**********************************************************/
#ifndef _XIO_H
#define _XIO_H_
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

/**********************************************************
*function    : int init_XIO();
*   returns  : nothing it is void
*   arguments: the input arguments
*Created by  : Jeff Venicx
*Date        :
*Description :
*Notes       :
**********************************************************/
enum complete {COMPLETE, INCOMPLETE};

int init_XIO();

void close_XIO();

int read_XIO(int fd);

int write_XIO(int fd);

void ioctl();

void transfer_XIO();
//setup and create vector table
void interput_setup();

#endif
