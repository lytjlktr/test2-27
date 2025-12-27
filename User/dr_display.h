#ifndef _display_dr_H
#define _display_dr_H

#include "main.h"

typedef struct //曲线参数
{
	float Draw_Buf[128];//曲线数据缓存
	float Draw_Min;//缓存数据的最小值  单位为百分比
	float Draw_Max;	//缓存数据的最大值
}_DrawCurve;//曲线参数


//字符转换,方便全局调用
extern char  LCD_CACHE[50];
//显示屏初始化
void DisplayInit(void);
//显示屏刷新显示
void DisplayRefresh(void) ;
//画点
void Oled_Draw_Point(uint8_t x,uint8_t y);
//显示字符串
void Oled_Disp_String(uint8_t x,uint8_t y,char const  *text);
//反显
void Interfacr_Shadow(uint8_t x,uint8_t y,uint8_t kuan,uint8_t gao);
//清除区域
void Clear_Disp(uint8_t x,uint8_t y,uint8_t kuan,uint8_t gao);
//清除滚动显示字符串的标志位
void OledScrollClear(void);
//滚动显示字符串
void OledScrollDisplay(uint8_t x,uint8_t y,char* str,uint8_t len);
//显示开机界面
void OledDisplayBootScreen(void);
//导入曲线数据
void WriteCurveData(_DrawCurve *Draw,float val);
//打印曲线
void OledDrawCurve(_DrawCurve Draw,uint8_t fllor,uint8_t upper);
//画线
void Display_Line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
//绘制矩形
void Display_REC(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
//绘制实心矩形
void Display_REC_Solid(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
//居中显示字符串
void OledCenteredDisplayStr(uint8_t x,uint8_t y ,uint8_t length,const char *str);
#endif

