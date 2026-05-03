#include "AllHeader.h"

int Encoder_Offset[4];
int Encoder_Now[4];

void IIC_Motor_Init(void)
{
    Wire.setSCL(PB10);
    Wire.setSDA(PB11);
    Wire.begin();
}

static int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    for (uint8_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    return Wire.endTransmission() != 0;
}

static int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return 1;

    Wire.requestFrom(addr, len);
    uint8_t i = 0;
    while (Wire.available() && i < len) {
        buf[i++] = Wire.read();
    }
    return (i == len) ? 0 : 1;
}

static void float_to_bytes(float f, uint8_t *bytes)
{
    memcpy(bytes, &f, sizeof(float));
}

static float char2float(char *p)
{
    float x;
    memcpy(&x, p, sizeof(float));
    return x;
}

void Set_motor_type(uint8_t data)
{
    i2cWrite(Motor_model_ADDR, MOTOR_TYPE_REG, 1, &data);
}

void Set_motor_deadzone(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xFF;
    buf[1] = data & 0xFF;
    i2cWrite(Motor_model_ADDR, MOTOR_DeadZONE_REG, 2, buf);
}

void Set_Pluse_line(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xFF;
    buf[1] = data & 0xFF;
    i2cWrite(Motor_model_ADDR, MOTOR_PluseLine_REG, 2, buf);
}

void Set_Pluse_Phase(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xFF;
    buf[1] = data & 0xFF;
    i2cWrite(Motor_model_ADDR, MOTOR_PlusePhase_REG, 2, buf);
}

void Set_Wheel_dis(float data)
{
    uint8_t bytes[4];
    float_to_bytes(data, bytes);
    i2cWrite(Motor_model_ADDR, WHEEL_DIA_REG, 4, bytes);
}

void control_speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    uint8_t speed[8];

    speed[0] = (m1 >> 8) & 0xFF;
    speed[1] = m1 & 0xFF;
    speed[2] = (m2 >> 8) & 0xFF;
    speed[3] = m2 & 0xFF;
    speed[4] = (m3 >> 8) & 0xFF;
    speed[5] = m3 & 0xFF;
    speed[6] = (m4 >> 8) & 0xFF;
    speed[7] = m4 & 0xFF;

    i2cWrite(Motor_model_ADDR, SPEED_Control_REG, 8, speed);
}

void control_pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    uint8_t pwm[8];

    pwm[0] = (m1 >> 8) & 0xFF;
    pwm[1] = m1 & 0xFF;
    pwm[2] = (m2 >> 8) & 0xFF;
    pwm[3] = m2 & 0xFF;
    pwm[4] = (m3 >> 8) & 0xFF;
    pwm[5] = m3 & 0xFF;
    pwm[6] = (m4 >> 8) & 0xFF;
    pwm[7] = m4 & 0xFF;

    i2cWrite(Motor_model_ADDR, PWM_Control_REG, 8, pwm);
}

void Read_10_Enconder(void)
{
    uint8_t buf[2];

    i2cRead(Motor_model_ADDR, READ_TEN_M1Enconer_REG, 2, buf);
    Encoder_Offset[0] = (buf[0] << 8) | buf[1];

    i2cRead(Motor_model_ADDR, READ_TEN_M2Enconer_REG, 2, buf);
    Encoder_Offset[1] = (buf[0] << 8) | buf[1];

    i2cRead(Motor_model_ADDR, READ_TEN_M3Enconer_REG, 2, buf);
    Encoder_Offset[2] = (buf[0] << 8) | buf[1];

    i2cRead(Motor_model_ADDR, READ_TEN_M4Enconer_REG, 2, buf);
    Encoder_Offset[3] = (buf[0] << 8) | buf[1];
}

void Read_ALL_Enconder(void)
{
    uint8_t buf[2], buf2[2];

    i2cRead(Motor_model_ADDR, READ_ALLHigh_M1_REG, 2, buf);
    i2cRead(Motor_model_ADDR, READ_ALLLOW_M1_REG, 2, buf2);
    Encoder_Now[0] = (buf[0] << 24) | (buf[1] << 16) | (buf2[0] << 8) | buf2[1];

    i2cRead(Motor_model_ADDR, READ_ALLHigh_M2_REG, 2, buf);
    i2cRead(Motor_model_ADDR, READ_ALLLOW_M2_REG, 2, buf2);
    Encoder_Now[1] = (buf[0] << 24) | (buf[1] << 16) | (buf2[0] << 8) | buf2[1];

    i2cRead(Motor_model_ADDR, READ_ALLHigh_M3_REG, 2, buf);
    i2cRead(Motor_model_ADDR, READ_ALLLOW_M3_REG, 2, buf2);
    Encoder_Now[2] = (buf[0] << 24) | (buf[1] << 16) | (buf2[0] << 8) | buf2[1];

    i2cRead(Motor_model_ADDR, READ_ALLHigh_M4_REG, 2, buf);
    i2cRead(Motor_model_ADDR, READ_ALLLOW_M4_REG, 2, buf2);
    Encoder_Now[3] = (buf[0] << 24) | (buf[1] << 16) | (buf2[0] << 8) | buf2[1];
}
