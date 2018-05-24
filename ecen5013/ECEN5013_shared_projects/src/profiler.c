/**********************************************************
* Name: profiler.c
*
* Date: 10/7/2016
*
* Description: This module contains implementations for the
* profiler
*
* Author: Ben Heberlein & Jeff Venicx
*
***********************************************************/

#ifdef FRDM
#include "uart.h"
#include "MKL25Z4.h"
#endif

#include "memory.h"
#include <sys/time.h>
#include "log.h"
#include "data.h"
#include "circbuf.h"
#include <time.h>

#include "stdlib.h"
#include "string.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


//binary bits for PRESCALE starting at 000 = 1 ... 111=128
#define PRESCALE 111
//decimal translation of PRESCALE
#define PRESCALE_MULTI 128

void delay(int milliseconds) {
    long pause;
    clock_t now,then;

    pause = milliseconds*(CLOCKS_PER_SEC/1000);
    now = then = clock();
    while( (now-then) < pause )
        now = clock();
}


void profile_memory(uint32_t length) {

		int32_t test;
		uint8_t time_buffer[32];
		int message_length;
		uint16_t cycles_128;

		uint8_t src[length];
		uint8_t dst[length];

		static struct timeval start;
		static struct timeval stop;
		struct timeval result;

		/*benchmark my_memmove*************************/
	#ifdef FRDM
		TPM1_CNT = 0;
		my_memmove(src, dst, length);
		cycles_128 = (uint16_t)TPM1_CNT;
		// Between this multiplication and the division of SystemCoreClock
		// test should be a value in uSeconds
		test = (cycles_128*PRESCALE_MULTI)*1000;
		//test = test/48;
		// Use SystemCoreClock for accuracy
		test = test/(SystemCoreClock/1000);

		message_length = my_itoa(time_buffer, test, 10);
	#else
			gettimeofday(&start, NULL);

			my_memmove(src, dst, length);

			gettimeofday(&stop, NULL);
			profile_convert_time(&result, &stop, &start);
			message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

	#endif
		Log3("my_memove time usec:", 20);
		Log0(time_buffer, message_length);

		//benchmark memmove*********************
	#ifdef FRDM
		TPM1_CNT = 0;

		memmove(dst, src, length);

		cycles_128 = (uint16_t)TPM1_CNT;
		// Between this multiplication and the division of SystemCoreClock
		// test should be a value in uSeconds
		test = (cycles_128*PRESCALE_MULTI)*1000;
		//test = test/48;
		// Use SystemCoreClock for accuracy
		test = test/(SystemCoreClock/1000);

		message_length = my_itoa(time_buffer, test, 10);
	#else

		gettimeofday(&start, NULL);

		memmove(dst, src, length);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

	#endif
		Log3("memmove time usec:", 18);
		Log0(time_buffer, message_length);

		//benchmark my_memzero******************
	#ifdef FRDM
		TPM1_CNT = 0;

		my_memzero(src, length);

		cycles_128 = (uint16_t)TPM1_CNT;
		// Between this multiplication and the division of SystemCoreClock
		// test should be a value in uSeconds
		test = (cycles_128*PRESCALE_MULTI)*1000;
		//test = test/48;
		// Use SystemCoreClock for accuracy
		test = test/(SystemCoreClock/1000);

		message_length = my_itoa(time_buffer, test, 10);
	#else

		gettimeofday(&start, NULL);

		my_memzero(src, length);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

	#endif
		Log3("my_memzero time usec:", 21);
		Log0(time_buffer, message_length);

		//benchmark memset for 0 *******************
	#ifdef FRDM
		TPM1_CNT = 0;

		memset(src, 0, length);

		cycles_128 = (uint16_t)TPM1_CNT;
		// Between this multiplication and the division of SystemCoreClock
		// test should be a value in uSeconds
		test = (cycles_128*PRESCALE_MULTI)*1000;
		//test = test/48;
		// Use SystemCoreClock for accuracy
		test = test/(SystemCoreClock/1000);

		message_length = my_itoa(time_buffer, test, 10);
	#else

		gettimeofday(&start, NULL);

		memset(src, 0, length);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

	#endif
		Log3("memset time usec:", 17);
		Log0(time_buffer, message_length);

		//benchmark my_reverse********************
	#ifdef FRDM
		TPM1_CNT = 0;

		my_reverse(src, length);

		cycles_128 = (uint16_t)TPM1_CNT;
		// Between this multiplication and the division of SystemCoreClock
		// test should be a value in uSeconds
		test = (cycles_128*PRESCALE_MULTI)*1000;
		//test = test/48;
		// Use SystemCoreClock for accuracy
		test = test/(SystemCoreClock/1000);

		message_length = my_itoa(time_buffer, test, 10);
	#else

		gettimeofday(&start, NULL);

		my_reverse(src, length);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

	#endif
		Log3("my_reverse time usec:", 21);
		Log0(time_buffer, message_length);

		// Benchmark reverse but how?
		// Output all profile data with log.h functions
}

void profile_data () {
		int message_length;
		uint16_t cycles_128;
		uint32_t length = 16;
		uint8_t buffer[32];
		int32_t interger = 12345;
		int32_t test;
		uint8_t time_buffer[32];

		float flt = 123123213121.234;

		static struct timeval start;
		static struct timeval stop;
		struct timeval result;


		//benchmark my_itoa*********************
	#ifdef FRDM
		TPM1_CNT = 0;

		my_itoa(buffer, interger, 10);

		cycles_128 = (uint16_t)TPM1_CNT;
		// Between this multiplication and the division of SystemCoreClock
		// test should be a value in uSeconds
		test = (cycles_128*PRESCALE_MULTI)*1000;
		//test = test/48;
		// Use SystemCoreClock for accuracy
		test = test/(SystemCoreClock/1000);

		message_length = my_itoa(time_buffer, test, 10);
	#else
		gettimeofday(&start, NULL);

		my_itoa(buffer, interger, 10);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

	#endif
		Log3("my_itoa time usec:", 18);
		Log0(time_buffer, message_length);

		//benchmark itoa*********************
	#ifdef BBB
		gettimeofday(&start, NULL);

		//itoa(interger, buffer, 10);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

		Log3("itoa time usec:", 15);
		Log0(time_buffer, message_length);

	#endif

		//benchmark my_ftoa**********************
	#ifdef FRDM
		TPM1_CNT = 0;

		my_ftoa(flt, buffer);

		cycles_128 = (uint16_t)TPM1_CNT;
		// Between this multiplication and the division of SystemCoreClock
		// test should be a value in uSeconds
		test = (cycles_128*PRESCALE_MULTI)*1000;
		//test = test/48;
		// Use SystemCoreClock for accuracy
		test = test/(SystemCoreClock/1000);

		message_length = my_itoa(time_buffer, test, 10);
	#else
		gettimeofday(&start, NULL);

		my_ftoa(flt, buffer);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

	#endif
		Log3("my_ftoa time usec:", 18);
		Log0(time_buffer, message_length);

		//benchmark ftoa BBB****************************
	#ifdef BBB
		gettimeofday(&start, NULL);

		//ftoa(flt, buffer);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

		Log3("ftoa time usec:", 15);
		Log0(time_buffer, message_length);
	#endif

		//benchmark my_atoi*********************
	#ifdef FRDM
		TPM1_CNT = 0;

		interger = my_atoi(buffer);

		cycles_128 = (uint16_t)TPM1_CNT;
		// Between this multiplication and the division of SystemCoreClock
		// test should be a value in uSeconds
		test = (cycles_128*PRESCALE_MULTI)*1000;
		//test = test/48;
		// Use SystemCoreClock for accuracy
		test = test/(SystemCoreClock/1000);

		message_length = my_itoa(time_buffer, test, 10);
	#else
		gettimeofday(&start, NULL);

		interger = my_atoi(buffer);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

	#endif
		Log3("my_atoi time usec:", 18);
		Log0(time_buffer, message_length);


			//benchmark atoi**********************
	#ifdef BBB
		gettimeofday(&start, NULL);

		//interger = my_atoi(buffer);

		gettimeofday(&stop, NULL);
		profile_convert_time(&result, &stop, &start);
		message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

		Log3("atoi time usec:", 15);
		Log0(time_buffer, message_length);
	#endif
}

void profile_malloc(uint32_t num_bytes) {

	int32_t test;
	uint8_t time_buffer[32];
	int message_length;
	uint16_t cycles_128;

	static struct timeval start;
	static struct timeval stop;
	struct timeval result;

#ifdef FRDM
	TPM1_CNT = 0;
	uint8_t *ptr;
	ptr = (uint8_t *) malloc(num_bytes);

	cycles_128 = (uint16_t)TPM1_CNT;
	// Between this multiplication and the division of SystemCoreClock
	// test should be a value in uSeconds
	test = (cycles_128*PRESCALE_MULTI)*1000;
	//test = test/48;
	// Use SystemCoreClock for accuracy
	test = test/(SystemCoreClock/1000);

	message_length = my_itoa(time_buffer, test, 10);
	free(ptr);
#else
	gettimeofday(&start, NULL);

	malloc(num_bytes);

	gettimeofday(&stop, NULL);
	profile_convert_time(&result, &stop, &start);
	message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);

#endif
	Log3("malloc time usec:", 17);
	Log0(time_buffer, message_length);



}

void profile_circbuf() {

	uint8_t time_buffer[32];
	int message_length;
	uint16_t cycles_128;
	int32_t test;

	circbuf_t *cb;
	cb = circbuf_initialize(10);

	static struct timeval start;
	static struct timeval stop;
	struct timeval result;

	/*benchmark circbuf_add_item*************************/
#ifdef FRDM
	TPM1_CNT = 0;

	circbuf_add_item(32, cb);

	cycles_128 = (uint16_t)TPM1_CNT;
	// Between this multiplication and the division of SystemCoreClock
	// test should be a value in uSeconds
	test = (cycles_128*PRESCALE_MULTI)*1000;
	//test = test/48;
	// Use SystemCoreClock for accuracy
	test = test/(SystemCoreClock/1000);

	message_length = my_itoa(time_buffer, test, 10);
#else
	gettimeofday(&start, NULL);

	circbuf_add_item(32, cb);

	gettimeofday(&stop, NULL);
	profile_convert_time(&result, &stop, &start);
	message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);
#endif
	Log3("add item time usec:", 19);
	Log0(time_buffer, message_length);

	/*benchmark ircbuf_remove_item *************************/
#ifdef FRDM
	TPM1_CNT = 0;

	circbuf_remove_item(cb);

	cycles_128 = (uint16_t)TPM1_CNT;
	// Between this multiplication and the division of SystemCoreClock
	// test should be a value in uSeconds
	test = (cycles_128*PRESCALE_MULTI)*1000;
	//test = test/48;
	// Use SystemCoreClock for accuracy
	test = test/(SystemCoreClock/1000);

	message_length = my_itoa(time_buffer, test, 10);
#else
	gettimeofday(&start, NULL);

	circbuf_remove_item(cb);

	gettimeofday(&stop, NULL);
	profile_convert_time(&result, &stop, &start);
	message_length = my_itoa(time_buffer, (int32_t)result.tv_usec, 10);
#endif
	Log3("remove item time usec:", 22);
	Log0(time_buffer, message_length);


	circbuf_destroy(cb);
}

void profile_log() {
#ifdef FRDM

	int32_t test;
	uint8_t time_buffer[32];
	int message_length;
	uint16_t cycles_128;

	TPM1_CNT = 0;

	Log0("test log time", 13);

	cycles_128 = (uint16_t)TPM1_CNT;
	// Between this multiplication and the division of SystemCoreClock
	// test should be a value in uSeconds
	test = (cycles_128*PRESCALE_MULTI)*1000;
	//test = test/48;
	// Use SystemCoreClock for accuracy
	test = test/(SystemCoreClock/1000);

	message_length = my_itoa(time_buffer, test, 10);

	Log3("Log0 time usec:", 15);
	Log0(time_buffer, message_length);
#endif
}

void profile_init(){
#ifdef FRDM
	//set clock gate for TPM1
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;

	//set clock mode and prescale factor
	TPM_SC_REG(TPM1) = TPM_SC_CMOD(1) | TPM_SC_PS(PRESCALE);

	//set mod number for max
	TPM_MOD_REG(TPM1) = 65535;

	uint32_t ssc = SystemCoreClock;
#endif
}

int profile_convert_time(struct timeval *result, struct timeval *x, struct timeval *y){

	/* Perform the carry for the later subtraction by updating y. */
   if (x->tv_usec < y->tv_usec) {
     int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
     y->tv_usec -= 1000000 * nsec;
     y->tv_sec += nsec;
   }
   if (x->tv_usec - y->tv_usec > 1000000) {
     int nsec = (x->tv_usec - y->tv_usec) / 1000000;
     y->tv_usec += 1000000 * nsec;
     y->tv_sec -= nsec;
   }

   /* Compute the time remaining to wait.
      tv_usec is certainly positive. */
   result->tv_sec = x->tv_sec - y->tv_sec;
   result->tv_usec = x->tv_usec - y->tv_usec;

   /* Return 1 if result is negative. */
   return 0;
}

void profile_all() {
	profile_init();

	Log0("profiler start", 14);

	Log0("10 bytes", 8);
	profile_memory(10);

	Log0("100 bytes", 8);
	profile_memory(100);

	Log0("1000 bytes", 9);
	profile_memory(1000);

	Log0("5000 bytes", 10);
	profile_memory(5000);

	profile_data();
/*
	Log0("malloc 10 bytes", 15);
	profile_malloc(10);

	Log0("malloc 100 bytes", 16);
	profile_malloc(100);

	Log0("malloc 500 bytes", 16);
	profile_malloc(500);

	Log0("malloc 1000 bytes", 17);
	profile_malloc(1000);

	profile_circbuf();

	profile_log();
*/
}
