/**********************************************************
* Name: timer.c
*
* Date: 10/4/2016
*
* Description: This module contains implementations for
* timer functions
*
* Author: Ben Heberlein
*
***********************************************************/
#include "MKL25Z4.h"
#include <stdint.h>
#include <timer.h>

#define TPM_MODULO 65535
#define TPM_PRESCALE 0

uint8_t TPM_DUTY = 50;
uint8_t RED_EN = 1;
uint8_t GRN_EN = 1;
uint8_t BLUE_EN = 1;

uint8_t init_timer() {

	// Set clock gates for PORTB and PORTD
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;
	// select clock source as MCGFLLCLK for both 0 and 2
	SIM_SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1);
	// Set clock gates for TPM0 and TPM2
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK;

	PORTB_PCR18 = PORT_PCR_MUX(3);
	PORTB_PCR19 |= PORT_PCR_MUX(3);
	PORTD_PCR1 |= PORT_PCR_MUX(4);

	// Modulo value to count to
	TPM_MOD_REG(TPM0) = TPM_MODULO;
	TPM_MOD_REG(TPM2) = TPM_MODULO;

	// Set clock mode and prescale
	TPM_SC_REG(TPM0) = TPM_SC_CMOD(1) | TPM_SC_PS(TPM_PRESCALE);// |  0x00000040;
	TPM_SC_REG(TPM2) = TPM_SC_CMOD(1) | TPM_SC_PS(TPM_PRESCALE);// |  0x00000040;

	// Set up channels for toggle output compare
	TPM_CnSC_REG(TPM0,1) = TPM_CnSC_ELSA(1) | TPM_CnSC_MSB(1);
	TPM_CnSC_REG(TPM2,0) = TPM_CnSC_ELSA(1) | TPM_CnSC_MSB(1);
	TPM_CnSC_REG(TPM2,1) = TPM_CnSC_ELSA(1) | TPM_CnSC_MSB(1);

	// Set match values
	TPM_CnV_REG(TPM0,1) = TPM_DUTY * TPM_MODULO / 100;
	TPM_CnV_REG(TPM2,0) = TPM_DUTY * TPM_MODULO / 100;
	TPM_CnV_REG(TPM2,1) = TPM_DUTY * TPM_MODULO / 100;
}

uint8_t get_duty() {
	return TPM_DUTY;
}

uint8_t set_duty(uint8_t duty) {
	if (duty >= 0 && duty <= 100) {
		// Set match values
		TPM_CnV_REG(TPM0,1) = duty * TPM_MODULO / 100 * BLUE_EN;
		TPM_CnV_REG(TPM2,0) = duty * TPM_MODULO / 100 * RED_EN;
		TPM_CnV_REG(TPM2,1) = duty * TPM_MODULO / 100 * GRN_EN;
		TPM_DUTY = duty;
		return 0;
	} else {
		return -1;
	}
}

uint8_t change_duty(int8_t change) {
	if (TPM_DUTY + change >= 0 && TPM_DUTY + change <= 100) {
		set_duty(TPM_DUTY + change);
		return 0;
	} else {
		return -1;
	}
}

uint8_t toggle_led(led_t led) {
	if (led == RED) {
		RED_EN = !RED_EN;
	} else if (led == GREEN) {
		GRN_EN = !GRN_EN;
	} else if (led == BLUE) {
		BLUE_EN = !BLUE_EN;
	}
	change_duty(0);
}

uint8_t color_led(led_t led) {
	if (led == RED) {
		RED_EN = 1;
		GRN_EN = 0;
		BLUE_EN = 0;
	} else if (led == GREEN) {
		RED_EN = 0;
		GRN_EN = 1;
		BLUE_EN = 0;
	} else if (led == BLUE) {
		RED_EN = 0;
		BLUE_EN = 1;
		GRN_EN = 0;
	} else if (led == YELLOW) {
		RED_EN = 1;
		BLUE_EN = 0;
		GRN_EN = 1;
	} else if (led == CYAN) {
		RED_EN = 0;
		GRN_EN = 1;
		BLUE_EN = 1;
	} else if (led == MAGENTA) {
		RED_EN = 1;
		GRN_EN = 0;
		BLUE_EN = 1;
	} else if (led == WHITE) {
		RED_EN = 1;
		GRN_EN = 1;
		BLUE_EN = 1;
	} else return -1;

	change_duty(0);
	return 0;
}

uint8_t led_routine(uint32_t slow) {
	set_duty(0);
	uint8_t i = 3;
	while(i--){
		RED_EN = 0;
		GRN_EN = 0;
		BLUE_EN = 0;
		change_duty(0);
		RED_EN = 1;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < slow; j++) {
				__NOP;
			}
			change_duty(5);
		}
		GRN_EN = 1;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < slow; j++) {
				__NOP;
			}
			change_duty(-5);
		}
		BLUE_EN = 1;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < slow; j++) {
				__NOP;
			}
			change_duty(5);
		}
		RED_EN = 0;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < slow; j++) {
				__NOP;
			}
			change_duty(-5);
		}
		RED_EN = 1;
		GRN_EN = 0;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < slow; j++) {
					__NOP;
			}
			change_duty(5);
		}
		RED_EN = 0;
		GRN_EN = 0;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < slow; j++) {
				__NOP;
			}
			change_duty(-5);
		}
		BLUE_EN = 0;
		GRN_EN = 1;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < slow; j++) {
				__NOP;
			}
			change_duty(5);
		}
		RED_EN = 0;
		GRN_EN = 0;
		BLUE_EN = 0;
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < slow; j++) {
				__NOP;
			}
			change_duty(-5);
		}
	}

	for (int j = 0; j < slow; j++) {
		__NOP;
	}

	RED_EN = 1;
	BLUE_EN = 1;
	GRN_EN = 1;
	set_duty(50);

	return 0;
}
