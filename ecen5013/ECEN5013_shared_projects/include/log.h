/**********************************************************
* Name: log.h
*
* Date: 10/3/2016
*
* Description: This module contains the definitions for
* several log functions in the UART logger.
*
* Author: Ben Heberlein
*
***********************************************************/

#ifndef LOG_H
#define LOG_H

#include <stdint.h>

void Log0(uint8_t *data, int32_t length);

void Log1(uint8_t *data, int32_t length, uint8_t *param, int32_t param_size);

void Log3(uint8_t *data, int32_t length);

#endif
