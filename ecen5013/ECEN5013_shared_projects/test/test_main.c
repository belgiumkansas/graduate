/**********************************************************
* Name: test_main.c
*
* Date: 9/14/16
*
* Description:
* This file holds the main routine for
* the test executable.
*
* Author: Ben Heberlein
*
***********************************************************/

#include <stdio.h>
#include "minunit.h"
#include "test_all.h"

int tests_run = 0;

char *all_tests() {
    mu_run_test_all(all_tests_data);
    mu_run_test_all(all_tests_memory);
    //TODO proper circbuf test integration
    all_tests_circbuf();
    //...
    return NULL;
}

int main() {
     char *result = NULL;
     result = all_tests();
     if (result != NULL) {
        printf("TEST FAILED\n");
        printf("%s\n", result);
     } else {
        printf("ALL TESTS PASSED\n");
     }
     printf("Tests run: %d\n", tests_run);

     return 0;
}
