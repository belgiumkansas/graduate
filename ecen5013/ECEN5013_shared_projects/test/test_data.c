/**********************************************************
* Name: test_data.c
*
* Date: 9/14/16
*
* Description:
* This file contains unit tests for several data
* manipulation functions
*
* Author: Ben Heberlein
*
***********************************************************/

#include <stdint.h>
#include <stdio.h>
#include "data.h"
#include "minunit.h"

static char *test_my_itoa() {
    printf("Testing my_itoa\n");

    // Error strings
    static char *error1 = "Failed to convert decimal number";
    static char *error2 = "Failed to convert negative number";
    static char *error3 = "Failed to convert binary number";
    static char *error4 = "Failed to convert hex number";

    int8_t *str;
    int32_t data = 104;
    int32_t base = 10;
    int8_t *ret = my_itoa(str, data, base);

    mu_assert(error1, *ret == '1');
    mu_assert(error1, *(ret+1) == '0');
    mu_assert(error1, *(ret+2) == '4');

    data = -104;

    ret = my_itoa(str, data, base);

    mu_assert(error2, *ret == '-');
    mu_assert(error2, *(ret+1) == '1');
    mu_assert(error2, *(ret+2) == '0');
    mu_assert(error2, *(ret+3) == '4');

    data = 0xA;
    base = 2;

    ret = my_itoa(str, data, base);

    mu_assert(error3, *ret == '1');
    mu_assert(error3, *(ret+1) == '0');
    mu_assert(error3, *(ret+2) == '1');
    mu_assert(error3, *(ret+3) == '0');

    data = 0xABC;
    base = 16;

    ret = my_itoa(str, data, base);

    mu_assert(error4, *ret == 'A');
    mu_assert(error4, *(ret+1) == 'B');
    mu_assert(error4, *(ret+2) == 'C');

    return NULL;
}

static char *test_my_atoi() {
    printf("Testing my_atoa\n");

    // Error strings
    static char *error1 = "Failed to convert positive integer";
    static char *error2 = "Failed to convert negative integer";
    static char *error3 = "Failed to convert zero";
    static char *error4 = "Should return 0 if input string is not a number";
    static char *error5 = "Should return 0 if input number is too big or too small";
    static char *error6 = "Should be able to handle leading zeros";

    mu_assert(error1, my_atoi("100") == 100);
    mu_assert(error2, my_atoi("-100") == -100);
    mu_assert(error3, my_atoi("0") == 0);
    mu_assert(error6, my_atoi("0000") == 0);
    mu_assert(error3, my_atoi("-0") == 0);
    mu_assert(error4, my_atoi("abc") == 0);
    mu_assert(error4, my_atoi("123abc") == 0);
    mu_assert(error5, my_atoi("3000000000") == 0);
    mu_assert(error5, my_atoi("-3000000000") == 0);
    mu_assert(error6, my_atoi("000123") == 123);
    mu_assert(error6, my_atoi("-000123") == -123);

    return NULL;
}

static char *test_dump_memory() {
    printf("Testing dump_memory\n");

    int32_t buffer_length = 100;
    uint8_t buffer[buffer_length];

    // Initialize data
    for (uint32_t i = 0; i < buffer_length; i++) {
        buffer[i] = i;
    }

    printf("The following command should print\n"
           "00 01 02 03 04 05 06 07 08 09 0A\n"
           "0B 0C 0D 0E 0F 10 11 12 13 14 15\n");
    dump_memory(buffer, 22);

    printf("The following command should print\n"
           "16 17 18 19 1A 1B 1C 1D 1E 1F 20\n");
    dump_memory(buffer+22, 11);

    printf("The following command should print 00\n");
    dump_memory(buffer, 1);

    return NULL;
}

static char *test_big_to_little() {
    printf("Testing bit_to_little\n");

    static char *error1 = "Failed to convert a 1 byte number";
    static char *error2 = "Failed to convert a 2 byte number";
    static char *error3 = "Failed to convert a 3 byte number";
    static char *error4 = "Failed to convert a 4 byte number";
    static char *error5 = "Failed to convert an 8 byte number";

    mu_assert(error1, big_to_little(0x0000000A) == 0x0A000000);
    mu_assert(error2, big_to_little(0x000000AB) == 0xAB000000);
    mu_assert(error3, big_to_little(0x00000ABC) == 0xBC0A0000);
    mu_assert(error4, big_to_little(0xABCD) == 0xCDAB0000);
    mu_assert(error5, big_to_little(0xABCDEF12) == 0x12EFCDAB);

    return NULL;
}

static char *test_little_to_big() {
    printf("Testing little_to_big\n");

    static char *error1 = "Failed to convert a 1 byte number";
    static char *error2 = "Failed to convert a 2 byte number";
    static char *error3 = "Failed to convert a 3 byte number";
    static char *error4 = "Failed to convert a 4 byte number";
    static char *error5 = "Failed to convert an 8 byte number";

    mu_assert(error1, little_to_big(0xA0000000) == 0x000000A0);
    mu_assert(error2, little_to_big(0xAB000000) == 0x000000AB);
    mu_assert(error3, little_to_big(0xBC0A0000) == 0x00000ABC);
    mu_assert(error4, little_to_big(0xCDAB0000) == 0xABCD);
    mu_assert(error5, little_to_big(0x21EFCDAB) == 0xABCDEF21);

    return NULL;
}

char *all_tests_data() {
    mu_run_test(test_my_itoa);
    mu_run_test(test_my_atoi);
    mu_run_test(test_dump_memory);
    mu_run_test(test_big_to_little);
    mu_run_test(test_little_to_big);
    return NULL;
}
