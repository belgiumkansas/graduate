/**********************************************************
* Name: SPI.c
*
* Date: 10/1/2016
*
* Description: This module contains implementations for
* SPI functions
*
* Author: Jeff Venicx
*
***********************************************************/

#include <stdint.h>
#include <stdlib.h>
#include "log.h"

#ifdef FRDM
#include "MKL25Z4.h"
#endif 

#ifdef BBB
#include <unistd.h>
#include <stdio.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/spi/spidev.h>



unsigned char value, null=0x00;         //sending only a single char
uint8_t bits = 8, mode = 1;             //8-bits per word, SPI mode 2 for CPOL!!!
uint32_t speed = 1000000;               //Speed is 1 MHz

#endif


int spi_init(){
  
  #ifdef FRDM
	//turn on clock to D module
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

	//enable SPIO0
	SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;

	PORTC_PCR8 |= PORT_PCR_MUX(0x01);  //Set PTC8 to mux 1 [SPI0_PCS0]
	//PORTC_PCR4 = PORT_PCR_MUX(0x2);    //Set PTC4 to mux 2 [SPI0_PCS0]
	PORTC_PCR5 = PORT_PCR_MUX(0x2);    //Set PTC5 to mux 2 [SPI0_SCK]
	PORTC_PCR6 = PORT_PCR_MUX(0x2);    //Set PTC6 to mux 2 [SPI0_MOSI]
	PORTC_PCR7 = PORT_PCR_MUX(0x2);    //Set PTC7 to mux 2 [SPIO_MISO]

	// Set port C, pin 8 data direction to output
	PTC_BASE_PTR->PDDR |= 1<<8;

	//Enable SPI0
	SPI0_C1 = SPI_C1_SPE_MASK;

	//Set SPI0 to Master & SS pin to auto SS. now its general io
	SPI0_C1 |= SPI_C1_MSTR_MASK; //| SPI_C1_SSOE_MASK;

	//Master SS pin acts as slave select output
	SPI0_C2 = SPI_C2_MODFEN_MASK;

	//Set baud rate prescale divisor to 3 & set baud rate divisor to 2 for baud rate of 250 khz
	SPI0_BR = (SPI_BR_SPPR(0x02) | SPI_BR_SPR(0b0010));

	//set SS pin to high
	PTC_BASE_PTR->PSOR |= 1<<8;

 
  return 0;
  #endif
  
  #ifdef BBB
  static const char *device = "/dev/spidev1.0";
	int fd;
    
     //mode |= SPI_CPOL;
  
  if ((fd = open(device, O_RDWR))<0){
      Log0("SPI Error: Can't open device.",29);
      return -1;
      }
      
  if (ioctl(fd, SPI_IOC_WR_MODE, &mode)==-1){
      Log0("SPI: Can't set SPI mode.", 23);
      return -1;
      }
      
  if (ioctl(fd, SPI_IOC_RD_MODE, &mode)==-1){
      Log0("SPI: Can't get SPI mode.", 23);
      return -1;
   }
   if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
      Log0("SPI: Can't set bits per word.",29);
      return -1;
   }
   
   if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
      Log0("SPI: Can't get bits per word.",29);
      return -1;
   }
   
   if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
      Log0("SPI: Can't set max speed HZ",27);
      return -1;
   }
   
   if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
      Log0("SPI: Can't get max speed HZ.",28);
      return -1;
   }
  
  return fd;
  #endif
  
  
}





void spi_send_uint8(uint8_t data){

}




void SPI0_handler(void){



}
