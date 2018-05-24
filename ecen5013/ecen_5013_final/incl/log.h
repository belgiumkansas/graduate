/**********************************************************
* Name:
*
* Date:
*
* Description:
* (description of the module here)
*
* Author:
*
**********************************************************/
#ifndef _LOG_H
#define _LOG_H_
#include<stdbool.h>
/**********************************************************
*function    : void function(void arguments);
*   returns  : nothing it is void
*   arguments: the input arguments
*Created by  : Jeff Venicx
*Date        :
*Description :
*Notes       :
**********************************************************/

void log_char(char *data, int32_t length, bool newline);

void log_uint8_t(uint8_t *data , int32_t length, bool newline);

void log_hexdump(uint8_t *data , int32_t length, bool newline);

#endif
