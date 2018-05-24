/**********************************************************
* Name: test_memory.c
*
* Date: 9/14/16
*
* Description:
* This file contains unit tests for several memory
* management functions.
*
* Author: Ben Heberlein
*
**********************************************************/

#include "stdint.h"
#include "memory.h"
#include "minunit.h"
#include "stdio.h"

static char *test_my_memmove() {
    printf("Testing my_memmove\n");

    // Create errors to be passed up through functions
    static char *error1 = "Could not move memory properly on non-overlapping buffers";
    static char *error2 = "Successful memory move should return 0";
    static char *error3 = "Could not move memory properly on overlapping buffers";
    static char *error4 = "Overlapping memory move should preserve memory values";
    static char *error5 = "Move length of zero should return 0 with memory unchanged";

    // Create memory buffer
    uint32_t buffer_length = 100;
    uint8_t buffer[buffer_length];

    uint32_t first_length = buffer_length/2;
    uint32_t second_length = buffer_length/4;

    // Initialize data
    for (uint32_t i = 0; i < first_length; i++) {
        buffer[i] = i;
        buffer[i+first_length] = 0;
    }

    // Move normally
    int8_t error = my_memmove(buffer, buffer+first_length, first_length);
    if (error != -1) {
        for (uint32_t i = 0; i < first_length; i++) {
            mu_assert(error1, buffer[i] == buffer[i+first_length]);
        }
        mu_assert(error2, error == 0);
    }

    // Reinitialize data
    for (uint32_t i = 0; i < first_length; i++) {
        buffer[i] = i;
        buffer[i+first_length] = 0;
    }

 /*
    // Move overlapping
    error = my_memmove(buffer, buffer+second_length, first_length);
    if (error != -1) {
        for (uint32_t i = 0; i < first_length; i++) {
            mu_assert(error3, buffer[i + second_length] == i);
        }
    }

    error = my_memmove(buffer+second_length, buffer, first_length);
    if (error != -1) {
        for (uint32_t i = 0; i < first_length; i++) {
            mu_assert(error3, buffer[i] == i);
        }
    }
 */
    // Length 0 should preserve values
    error = my_memmove(buffer, buffer+first_length, 0);
    mu_assert(error5, error == 0);
    for (uint32_t i = 0; i < first_length; i++) {
        mu_assert(error4, buffer[i] == i);
        mu_assert(error4, buffer[i+first_length] == 0);
    }

    return NULL;
}

static char *test_my_memzero() {
    printf("Testing my_memzero\n");

    // Create memory buffer
    uint32_t buffer_length = 100;
    uint8_t buffer[buffer_length];

    uint32_t first_length = 10;

    // Initialize data
    for (uint32_t i = 0; i < buffer_length; i++) {
        buffer[i] = i;
    }

    int8_t error = my_memzero(buffer, first_length);
    if (error != -1) {
        for (uint32_t i = 0; i < first_length; i++) {
            mu_assert("Failed to zero out specified length of memory", buffer[i] == 0);
        }
        for (uint32_t i = first_length; i < buffer_length - first_length; i++) {
            mu_assert("Cleared memory outside of range.", buffer[i] == i);
        }
        mu_assert("Should return 0 for succesful memory clear", error == 0);
    }

    return NULL;
}

static char *test_my_reverse() {
    printf("Testing my_reverse\n");

    // Create memory buffer
    uint32_t buffer_length = 100;
    uint8_t buffer[buffer_length];

    uint32_t first_length = 10;

    // Initialize data counting every ten
    for (uint32_t i = 0; i < buffer_length; i++) {
        buffer[i] = i % 10;
    }

    int8_t error = my_reverse(buffer, first_length);
    if (error != -1) {
        for (uint32_t i = 0; i < first_length; i++) {
            mu_assert("Failed to flip data", buffer[i] == buffer[2*first_length-i-1]);
        }
        for (uint32_t i = first_length; i < buffer_length - first_length; i++) {
            mu_assert("Flipped memory outside of range.", buffer[i] == i % 10);
        }
        mu_assert("Should return 0 for succesful memory flip", error == 0);
    }

    return NULL;
}

char *all_tests_memory() {
    mu_run_test(test_my_memmove);
    mu_run_test(test_my_memzero);
    mu_run_test(test_my_reverse);
    return NULL;
}
