#include "AllHeader.h"

void BSP_init(void)
{
    USART1_init(115200);
    USART2_init(115200);
    RGB_Init();
    IIC_Motor_Init();
}
