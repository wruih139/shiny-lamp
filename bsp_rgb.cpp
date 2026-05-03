#include "AllHeader.h"

static Adafruit_NeoPixel strip(MAX_RGB, RGB_PIN, NEO_GRB + NEO_KHZ800);

void RGB_Init(void)
{
    strip.begin();
    strip.show();
}

void RGB_Update(void)
{
    strip.show();
}

void RGB_Set_Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < MAX_RGB) {
        strip.setPixelColor(index, r, g, b);
        return;
    }
    if (index == RGB_CTRL_ALL) {
        for (uint8_t i = 0; i < MAX_RGB; i++) {
            strip.setPixelColor(i, r, g, b);
        }
    }
}

void RGB_Set_Color_U32(uint8_t index, uint32_t color)
{
    if (index < MAX_RGB) {
        strip.setPixelColor(index, color);
        return;
    }
    if (index == RGB_CTRL_ALL) {
        for (uint8_t i = 0; i < MAX_RGB; i++) {
            strip.setPixelColor(i, color);
        }
    }
}

void RGB_Clear(void)
{
    strip.clear();
    strip.show();
}
