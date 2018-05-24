/**********************************************************
* Name: nRF24L01.c
*
* Date: 10/1/2016
*
* Description: This module contains implementations for
* nRF24L01 interface functions
*
* Author: Jeff Venicx
*
***********************************************************/


#include "nRF24L01.h"
#include "SPI.h"
#include <stdint.h>
#include <stdlib.h>


#ifdef FRDM
#include "MKL25Z4.h"
#endif

#ifdef BBB

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

static uint8_t bits = 8;
static uint32_t speed = 250000;

#endif


#define NRF_SS_ENABLE (PTC_BASE_PTR->PCOR |= 1<<8)
#define NRF_SS_DISABLE (PTC_BASE_PTR->PSOR |= 1<<8)

#define SPI_WAIT_SPTEF (!(SPI0_S & SPI_S_SPTEF_MASK))
#define SPI_WAIT_SPRF (!(SPI0_S & SPI_S_SPRF_MASK))



uint8_t nrf_get_status(int fd){
  #ifdef FRDM
	uint8_t sreg = 0;
	NRF_SS_ENABLE;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send NOP
	SPI0_D = NRF_NOP;

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	sreg = SPI0_D;
	NRF_SS_DISABLE;
	return sreg;

  #endif
  #ifdef BBB
  
    uint8_t tx[] = {NRF_NOP};               
    uint8_t rx[] = {0x00}; 
    
    struct spi_ioc_transfer tr;  
  	tr.tx_buf = tx;
  	tr.rx_buf = rx;
  	tr.len = 1;
  	tr.delay_usecs = 0;
  	tr.speed_hz = speed;
  	tr.bits_per_word = bits;
    
    ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   
    return rx[0];
     
  #endif
}

uint8_t nrf_get_config(int fd){
  #ifdef FRDM
	uint8_t sreg = 0;
	uint8_t creg = 0;
	NRF_SS_ENABLE;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send read config command
	SPI0_D = (NRF_RD_CMD|NRF_CONFIG_REG);

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store status register
	sreg = SPI0_D;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send NOP
	SPI0_D = NRF_NOP;

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store config register
	creg = SPI0_D;

	NRF_SS_DISABLE;
	return creg;
  #endif
  
  #ifdef BBB
  uint8_t tx[] = {NRF_RD_CMD|NRF_CONFIG_REG, NRF_NOP};               
  uint8_t rx[] = {0x00, 0x00}; 
    
    struct spi_ioc_transfer tr;  
  	tr.tx_buf = tx;
  	tr.rx_buf = rx;
  	tr.len = 2;
  	tr.delay_usecs = 0;
  	tr.speed_hz = speed;
  	tr.bits_per_word = bits;
    
   ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   
   return rx[1];
 
  #endif
}

uint8_t nrf_write_config(int fd, uint8_t val){
  #ifdef FRDM

	uint8_t sreg = 0, tmp;

	NRF_SS_ENABLE;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send write config command
	SPI0_D = (NRF_CONFIG_REG|NRF_WR_CMD);

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store status register
	sreg = SPI0_D;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send NOP
	SPI0_D = val;

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);
    tmp = SPI0_D;

	NRF_SS_DISABLE;
	return sreg;
  #endif
  
  #ifdef BBB
  
  uint8_t tx[] = {NRF_WR_CMD|NRF_CONFIG_REG, val};               
  uint8_t rx[] = {0x00, 0x00}; 
    
    struct spi_ioc_transfer tr;  
  	tr.tx_buf = tx;
  	tr.rx_buf = rx;
  	tr.len = 2;
  	tr.delay_usecs = 0;
  	tr.speed_hz = speed;
  	tr.bits_per_word = bits;
    
   ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   
   return rx[0];
   
  #endif
}

uint8_t nrf_read_tx_addr(int fd, uint8_t * address){
  #ifdef FRDM
	uint8_t sreg = 0;

	NRF_SS_ENABLE;

	while(SPI_WAIT_SPTEF);

	SPI0_D = (NRF_RD_CMD|NRF_TX_ADDR_REG);

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store status register
	sreg = SPI0_D;

	for(int i=0; i < 5; i++){
		//wait for tx to fill
		while(SPI_WAIT_SPTEF);

		//write nop code for clock ticks
		SPI0_D = NRF_NOP;

		//wait for rx to fill
		while(SPI_WAIT_SPRF);

		//write address
		address[i] = SPI0_D;
	}

	NRF_SS_DISABLE;
	return sreg;

  #endif
  #ifdef BBB
  
  
   uint8_t tx[] = {NRF_RD_CMD|NRF_TX_ADDR_REG, NRF_NOP, NRF_NOP, NRF_NOP, NRF_NOP, NRF_NOP};               
   uint8_t rx[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
    
    struct spi_ioc_transfer tr;  
  	tr.tx_buf = tx;
  	tr.rx_buf = rx;
  	tr.len = 6;
  	tr.delay_usecs = 0;
  	tr.speed_hz = speed;
  	tr.bits_per_word = bits;
    
   ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   
   for(int i = 0; i < 5; i++){
     address[i] = rx[i+1];
   }
   
   return rx[0]; 
 
  #endif
}

uint8_t nrf_write_tx_addr(int fd, uint8_t * address){
  #ifdef FRDM

	uint8_t sreg = 0, tmp;

	NRF_SS_ENABLE;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//transmit write command
	SPI0_D = (NRF_WR_CMD|NRF_TX_ADDR_REG);

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);
	sreg = SPI0_D;

	//transmit tx address
	for(int i=0 ; i<5; i++){
		while(SPI_WAIT_SPTEF);
		SPI0_D = address[i];

		while(SPI_WAIT_SPTEF);

		//wait for Read buffer to fill
		while(SPI_WAIT_SPRF);
		tmp = SPI0_D;
	}

	//send NOP command
	SPI0_D = NRF_NOP;

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);
    tmp = SPI0_D;

	NRF_SS_DISABLE;
    return tmp;

  #endif
  #ifdef BBB
  
   uint8_t tx[] = {NRF_WR_CMD|NRF_TX_ADDR_REG, address[0], address[1], address[2], address[3], address[4]};               
   uint8_t rx[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
    
    struct spi_ioc_transfer tr;  
  	tr.tx_buf = tx;
  	tr.rx_buf = rx;
  	tr.len = 6;
  	tr.delay_usecs = 0;
  	tr.speed_hz = speed;
  	tr.bits_per_word = bits;
    
   ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   
   return rx[0];
 
   
  #endif
}

uint8_t nrf_read_rf_setup(int fd){
  #ifdef FRDM

	uint8_t sreg,rfreg;

	NRF_SS_ENABLE;

	while(SPI_WAIT_SPTEF);

	SPI0_D = (NRF_RD_CMD|NRF_RF_SETUP_REG);

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store status register
	sreg = SPI0_D;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send NOP
	SPI0_D = NRF_NOP;

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store rf setup register
	rfreg = SPI0_D;


	NRF_SS_DISABLE;
	return rfreg;

  #endif
  #ifdef BBB
   uint8_t tx[] = {NRF_RD_CMD|NRF_RF_SETUP_REG, NRF_NOP};               
   uint8_t rx[] = {0x00, 0x00}; 
    
    struct spi_ioc_transfer tr;  
  	tr.tx_buf = tx;
  	tr.rx_buf = rx;
  	tr.len = 2;
  	tr.delay_usecs = 0;
  	tr.speed_hz = speed;
  	tr.bits_per_word = bits;
    
   ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   
   return rx[1];
  #endif
}

uint8_t nrf_write_rf_setup_power(int fd, uint8_t POWER_LEVEL){
  #ifdef FRDM

	uint8_t sreg = 0, tmp;

	NRF_SS_ENABLE;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send write config command
	SPI0_D = (NRF_WR_CMD|NRF_RF_SETUP_REG);

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store status register
	sreg = SPI0_D;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send power level
	SPI0_D = POWER_LEVEL;

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);
    tmp = SPI0_D;

	NRF_SS_DISABLE;
	return sreg;

  #endif
  #ifdef BBB
   
  uint8_t tx[] = {NRF_WR_CMD|NRF_RF_SETUP_REG, POWER_LEVEL};               
  uint8_t rx[] = {0x00, 0x00}; 
    
    struct spi_ioc_transfer tr;  
  	tr.tx_buf = tx;
  	tr.rx_buf = rx;
  	tr.len = 2;
  	tr.delay_usecs = 0;
  	tr.speed_hz = speed;
  	tr.bits_per_word = bits;
    
   ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   
   return rx[0];
   
   
  #endif
}

uint8_t nrf_get_fifo_status(int fd){
	#ifdef FRDM
  uint8_t sreg = 0;

	uint8_t fiforeg = 0;
	NRF_SS_ENABLE;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send read config command
	SPI0_D = (NRF_RD_CMD|NRF_FIFO_STATUS_REG);

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store status register
	sreg = SPI0_D;

	//check tx buffer
	while(SPI_WAIT_SPTEF);

	//send NOP
	SPI0_D = NRF_NOP;

	//wait for Read buffer to fill
	while(SPI_WAIT_SPRF);

	//store fifo_status register
	fiforeg = SPI0_D;

	NRF_SS_DISABLE;
	return fiforeg;

  #endif
  #ifdef BBB
  uint8_t tx[] = {NRF_RD_CMD|NRF_FIFO_STATUS_REG, NRF_NOP};               
  uint8_t rx[] = {0x00, 0x00}; 
    
    struct spi_ioc_transfer tr;  
  	tr.tx_buf = tx;
  	tr.rx_buf = rx;
  	tr.len = 2;
  	tr.delay_usecs = 0;
  	tr.speed_hz = speed;
  	tr.bits_per_word = bits;
    
   ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
   
   return rx[1];
 #endif

}


