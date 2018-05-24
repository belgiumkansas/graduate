/**********************************************************
* Name: minunit.h
* 
* Date: 9/14/16
*
* Description: This module contains the macros that define
* the MinUnit testing framework. Free for reuse at
* http://www.jera.com/techinfo/jtns/jtn002.html
*
* Author: Ben Heberlein
*
**********************************************************/

#ifndef MINUNIT_H
#define MINUNIT_H

#define mu_assert(message, test) do { if (!(test)) return message; } while (0)
#define mu_run_test(test) do { char *message = test(); tests_run++; \
                                if (message) return message; } while (0)
#define mu_run_test_all(test) do { char *message = test(); \
                                if (message) return message; } while(0)
extern int tests_run;

#endif
