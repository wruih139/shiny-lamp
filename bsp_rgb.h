#ifndef __BSP_RGB_H__
#define __BSP_RGB_H__

#include "AllHeader.h"

#define RGB_CTRL_ALL  0xFF
#define MAX_RGB       14
#define RGB_PIN       PA7

void RGB_Init(void);
void RGB_Update(void);

void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void RGB_Set_Color_U32(uint8_t index, uint32_t color);
void RGB_Clear(void);

#endif
