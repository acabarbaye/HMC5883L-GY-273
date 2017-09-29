/*
 * Logging.h
 *
 *  Created on: 13 Sep 2017
 *      Author: adrcab01
 */

#ifndef LOGGING_H_
#define LOGGING_H_
#include "mbed.h"

#define LOGGING_ENABLED 1
#ifdef LOGGING_ENABLED
#define LOG_DEBUG(message) printf("[DEBUG]: %s\r\n",message);
#define LOG_DEBUG_DOUBLE_VALUE(value_name,value)  printf("[DEBUG]: %s=%g\r\n",value_name,value);
#define LOG_DEBUG_INTEGER_VALUE(value_name,value)  printf("[DEBUG]: %s=%d\r\n",value_name,value);
#define LOG_DEBUG_HEXADECIMAL_VALUE(value_name,value)  printf("[DEBUG]: %s=%x\r\n",value_name,value);
#define LOG_TRACE(message) printf("[TRACE]: %s\r\n",message);
#define LOG_TRACE_DOUBLE_VALUE(value_name,value)  printf("[TRACE]: %s=%g\r\n",value_name,value);
#define LOG_TRACE_INTEGER_VALUE(value_name,value)  printf("[TRACE]: %s=%d\r\n",value_name,value);
#define LOG_TRACE_HEXADECIMAL_VALUE(value_name,value)  printf("[TRACE]: %s=%x\r\n",value_name,value);
#else
#define LOG_DEBUG(x)
#define LOG_DEBUG_DOUBLE_VALUE(value_name,value)
#define LOG_DEBUG_INTEGER_VALUE(value_name,value)
#define LOG_DEBUG_HEXADECIMAL_VALUE
#define LOG_TRACE(message)
#define LOG_TRACE_DOUBLE_VALUE(value_name,value)
#define LOG_TRACE_INTEGER_VALUE(value_name,value)
#define LOG_TRACE_HEXADECIMAL_VALUE(value_name,value)
#endif

#endif /* LOGGING_H_ */
