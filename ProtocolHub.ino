//K230 -> Board     4Way Motor Board -> Board   RGB Light -> Board
//TXD  -> PA3       SCL->PB10                    LED  -> PA7
//RXD  -> PA2       SDA->PB11

#include "AllHeader.h"

#define MOTOR_TYPE 4

ColorMode color_mode = COLOR_MODE_FOLLOW_LINE;
int Car_Auto_Drive = 0;

void setup()
{
    BSP_init();
    Set_Motor(MOTOR_TYPE);

    if (Car_Auto_Drive) {
        delay(1000); delay(1000); delay(1000); delay(1000); delay(1000);
        delay(1000); delay(1000); delay(1000); delay(1000); delay(1000);
    }
    printf("Init Succeed\r\n");
}

void loop()
{
    while (Serial2.available()) {
        Pto_Data_Receive(Serial2.read());
        lost_count = 50;
    }

    Pto_Loop();
    delay(10);
}
