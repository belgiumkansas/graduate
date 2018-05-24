/**********************************************************
* Name: timer.h
*
* Date: 10/4/16
*
* Description: This module defines the timer interface
*
* Author: Ben Heberlein
*
***********************************************************/

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

typedef enum led{RED,GREEN,BLUE,YELLOW,CYAN,MAGENTA,WHITE} led_t;

uint8_t init_timer();

uint8_t get_duty();

uint8_t set_duty(uint8_t duty);

uint8_t change_duty(int8_t change);

uint8_t toggle_led(led_t led);

uint8_t color_led(led_t led);

uint8_t led_routine(uint32_t slow);

#endif
