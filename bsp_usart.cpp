#include "AllHeader.h"

void USART1_init(unsigned long baudrate)
{
    Serial.begin(baudrate);
    while (!Serial);
}

void USART2_init(unsigned long baudrate)
{
    Serial2.begin(baudrate);
    while (!Serial2);
}

void USART1_Send_U8(uint8_t ch)
{
    Serial.write(ch);
}

void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    Serial.write(BufferPtr, Length);
}

void USART2_Send_U8(uint8_t ch)
{
    Serial2.write(ch);
}

void USART2_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
    Serial2.write(BufferPtr, Length);
}

extern "C" int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++) {
        Serial.write((uint8_t)ptr[i]);
    }
    return len;
}
