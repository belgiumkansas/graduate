/**********************************************************
* Name: nRF24L01.h
*
* Date: 10/1/2016
*
* Description: This module defines the nRF24L01 interface
* functions
*
* Author: Jeff Venicx
*
***********************************************************/
#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>

//nrf masks
#define NRF_NOP (0xFF)
#define NRF_PWR_UP (0x02)
#define NRF_EN_CRC (0x04)


//RF output tags
#define NRF_RF_18 (0x00)
#define NRF_RF_12 (0x02)
#define NRF_RF_6 (0x04)
#define NRF_RF_0 (0x06)

//nrf registers
#define NRF_CONFIG_REG (0x00)
#define NRF_TX_ADDR_REG (0x10)
#define NRF_RF_SETUP_REG (0x06)
#define NRF_FIFO_STATUS_REG (0x17)

//nrf commands
#define NRF_WR_CMD (0x20)
#define NRF_RD_CMD (0x00)

uint8_t nrf_get_status(int fd);

uint8_t nrf_get_config(int fd);

uint8_t nrf_write_config(int fd, uint8_t val);

uint8_t nrf_read_tx_addr(int fd, uint8_t * address);

uint8_t nrf_write_tx_addr(int fd, uint8_t * address);

uint8_t nrf_read_rf_setup(int fd);

uint8_t nrf_write_rf_setup_power(int fd, uint8_t POWER_LEVEL);


#endif
