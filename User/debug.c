/**
 * @file debug.c
 * @Co.  Cheez 
 * @author  Vecang
 * @brief debug  
 *! 需要打开微库
 *
 * 在cubemx里初始化串口后，在debug.h里定义相对的串口
 * 例如：EMLOG(LOG_DEBUG,"HELLO %d",val);
 *
 * @version 0.1
 * @date 2022-08-08
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "debug.h"
#include "usbd_cdc_if.h"

// void USB_printf(const char * fmt,...)
// {
// 		uint16_t len;
//     va_list args;
//     va_start(args, fmt);

//     // char buf[1 + vsnprintf(NULL, 0, fmt, arg)];
//     char buf[50] = {0};
//     vsnprintf((char*)buf, sizeof(buf), (char*)fmt, args);
//     va_end(args);
//     CDC_Transmit_FS((uint8_t*)buf,len);
// } 

void Debug(uint8_t *tx_buffer, uint16_t len)
{
    HAL_UART_Transmit(&DEBUG, tx_buffer, len, 1000);
}

char *EM_LOGLevelGet(E_LOGLEVEL level)
{
    char *getlevel = "UNKNOWN";
    switch (level)
    {
    case LOG_DEBUG:
        getlevel = "DEBUG";
        break;
    case LOG_INFO:
        getlevel = "INFO";
        break;
    case LOG_WARN:
        getlevel = "WARN";
        break;
    case LOG_ERROR:
        getlevel = "ERROR";
        break;
    default:
        getlevel = "UNKNOWN";
        break;
    }
    return getlevel;
}

void EM_LOG(E_LOGLEVEL Level, const char *FunName, const int Line, const char *fmt, ...)
{
#ifdef OPEN_LOG
    va_list arg;
    va_start(arg, fmt);

    // char buf[1 + vsnprintf(NULL, 0, fmt, arg)];
    char buf[50] = {0};
    vsnprintf(buf, sizeof(buf), fmt, arg);
    va_end(arg);
    if (Level >= LOG_LEVEL)
    {
        printf("[%-5s] [%s %4d]:%s \n", EM_LOGLevelGet(Level), FunName, Line, buf);
    }
#endif
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&DEBUG, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

