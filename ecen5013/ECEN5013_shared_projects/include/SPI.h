/**********************************************************
* Name: SPI.h
*
* Date: 11/1/16
*
* Description: This module defines the SPI interface
*
* Author: Jeff Venicx
*
***********************************************************/

#ifndef SPI_H
#define SPI_H

#include <stdint.h>

int spi_init();

void spi_send(char spiMsg);

uint8_t spi_send_command(uint8_t cmd);

void spi_send_uint8(uint8_t data);

uint8_t spi_read_uint8(void);

void spi_ss_on(void);

void spi_ss_off(void);

void spi_flush();


#endif
