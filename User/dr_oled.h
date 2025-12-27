#ifndef _dr_oled_H
#define _dr_oled_H

#include "main.h"

//OLED初始化
void OLED_Init(void);
//OLED全屏显示
void OLED_Diaplay(char displaybuf[][8]);
//关闭OLED显示 
void OLED_Display_Off(void);
//开启OLED显示
void OLED_Display_On(void);

#endif

