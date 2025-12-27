#ifndef __DEBUG_H__
#define __DEBUG_H__
#include <stdio.h>
#include <stdarg.h>
#include "main.h"

/*********************自定义配置***************************/

#define DEBUG huart1
#define OPEN_LOG 1          // 是否开启DEBUG
#define LOG_LEVEL LOG_DEBUG // DEBUG等级

/*********************************************************/

typedef enum
{
    LOG_DEBUG = 1,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
} E_LOGLEVEL;

void EM_LOG(E_LOGLEVEL Level, const char *FunName, const int Line, const char *fmt, ...);

#define EMLOG(level, fmt, ...) EM_LOG(level, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

extern UART_HandleTypeDef DEBUG;

#endif

