#include "app_motor.h"

#define DEBUG 0
#if DEBUG
    #define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)
#endif

#define KP   (12)
#define KI   (0)
#define KD   (0.1)

int pid_output, pid_output1 = 0;
int PWM_Car_Flag = 0;
int Wait_Flag = 0;
int SetTarget_Flag = 0;
int target_area = 0;

static float speed_lr = 0;
static float speed_fb = 0;
static float speed_spin = 0;
static int speed_L1_setup = 0;
static int speed_L2_setup = 0;
static int speed_R1_setup = 0;
static int speed_R2_setup = 0;

float PID_Calc(float actual_value)
{
    float output = 0;
    float error = actual_value;
    static float error_last;
    static float Integral;

    Integral += error;

    output = error * KP + KI * Integral + (error - error_last) * KD;

    error_last = error;

    return output;
}

static float Motion_Get_APB(void)
{
    return Car_APB;
}

void Set_Motor(int MOTOR_TYPE)
{
    if (MOTOR_TYPE == 1) {
        Set_motor_type(1);
        delay(100);
        Set_Pluse_Phase(30);
        delay(100);
        Set_Pluse_line(11);
        delay(100);
        Set_Wheel_dis(67.00);
        delay(100);
        Set_motor_deadzone(1900);
        delay(100);
    } else if (MOTOR_TYPE == 2) {
        Set_motor_type(2);
        delay(100);
        Set_Pluse_Phase(20);
        delay(100);
        Set_Pluse_line(13);
        delay(100);
        Set_Wheel_dis(48.00);
        delay(100);
        Set_motor_deadzone(1600);
        delay(100);
    } else if (MOTOR_TYPE == 3) {
        Set_motor_type(3);
        delay(100);
        Set_Pluse_Phase(45);
        delay(100);
        Set_Pluse_line(13);
        delay(100);
        Set_Wheel_dis(68.00);
        delay(100);
        Set_motor_deadzone(1250);
        delay(100);
    } else if (MOTOR_TYPE == 4) {
        Set_motor_type(4);
        delay(100);
        Set_Pluse_Phase(48);
        delay(100);
        Set_motor_deadzone(1000);
        delay(100);
        PWM_Car_Flag = 1;
    } else if (MOTOR_TYPE == 5) {
        Set_motor_type(1);
        delay(100);
        Set_Pluse_Phase(40);
        delay(100);
        Set_Pluse_line(11);
        delay(100);
        Set_Wheel_dis(67.00);
        delay(100);
        Set_motor_deadzone(1900);
        delay(100);
    }
}

void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z)
{
    float robot_APB = Motion_Get_APB();
    speed_lr = 0;
    speed_fb = V_x;
    speed_spin = (V_z / 1000.0f) * robot_APB;

    if (V_x == 0 && V_y == 0 && V_z == 0) {
        control_pwm(0, 0, 0, 0);
        return;
    }

    speed_L1_setup = speed_fb - speed_spin;
    speed_L2_setup = speed_fb - speed_spin;
    speed_R1_setup = speed_fb + speed_spin;
    speed_R2_setup = speed_fb + speed_spin;

    if (speed_L1_setup > 1000)  speed_L1_setup = 1000;
    if (speed_L1_setup < -1000) speed_L1_setup = -1000;
    if (speed_L2_setup > 1000)  speed_L2_setup = 1000;
    if (speed_L2_setup < -1000) speed_L2_setup = -1000;
    if (speed_R1_setup > 1000)  speed_R1_setup = 1000;
    if (speed_R1_setup < -1000) speed_R1_setup = -1000;
    if (speed_R2_setup > 1000)  speed_R2_setup = 1000;
    if (speed_R2_setup < -1000) speed_R2_setup = -1000;

    if (PWM_Car_Flag) {
        control_pwm(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
    } else {
        control_speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
    }
}

void Visual_Line_Track(int x)
{
    if (!Wait_Flag) {
        delay(1000); delay(1000); delay(1000);
        Wait_Flag = 1;
        return;
    }

    int16_t err = 300 - (x + w / 2);
    DEBUG_PRINT("x:%d, CENTER:%d\r\n", x, 300);
    DEBUG_PRINT("err:%d\r\n", err);
    if (err < 5 && err > -5) err = 0;

    pid_output = PID_Calc(err);
    DEBUG_PRINT("pid_output:%d\r\n", pid_output);

    Motion_Car_Control(300, 0, pid_output);
}

void Color_Trace(int x, int w, int h)
{
    if (!Wait_Flag) {
        delay(1000); delay(1000); delay(1000);
        Wait_Flag = 1;
        return;
    }

    if (Wait_Flag && !SetTarget_Flag) {
        SetTarget_Flag = 1;
        target_area = w * h / 100;
    }

    int now_area = w * h / 100;

    int16_t err = 280 - x;
    DEBUG_PRINT("x:%d, CENTER:%d\r\n", x, 280);
    DEBUG_PRINT("err:%d\r\n", err);

    pid_output = PID_Calc(err);
    DEBUG_PRINT("pid_output:%d\r\n", pid_output);
    if (pid_output > 500)  pid_output = 500;
    if (pid_output < -500) pid_output = -500;

    DEBUG_PRINT("\r\n");

    int16_t err_area = target_area - now_area;
    DEBUG_PRINT("now_area:%d, target_area:%d\r\n", now_area, target_area);
    DEBUG_PRINT("err_area:%d\r\n", err_area);

    if (err_area < 10 && err_area > -10) err_area = 0;
    pid_output1 = PID_Calc(err_area);
    DEBUG_PRINT("pid_output1:%d\r\n", pid_output1);
    if (pid_output1 > 200)  pid_output1 = 200;
    if (pid_output1 < -200) pid_output1 = -200;

    DEBUG_PRINT("\r\n");

    Motion_Car_Control(pid_output1, 0, pid_output);
}

void QRCode_Action(const char* msg)
{
    if (strcmp(msg, "FORWARD") == 0 || strcmp(msg, "forward") == 0) {
        DEBUG_PRINT("FORWARD!\r\n");
        Motion_Car_Control(200, 0, 0);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "BACKWARD") == 0 || strcmp(msg, "backward") == 0) {
        DEBUG_PRINT("BACKWARD!\r\n");
        Motion_Car_Control(-200, 0, 0);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "LEFT") == 0 || strcmp(msg, "left") == 0) {
        DEBUG_PRINT("LEFT!\r\n");
        Motion_Car_Control(200, 0, 500);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "RIGHT") == 0 || strcmp(msg, "right") == 0) {
        DEBUG_PRINT("RIGHT!\r\n");
        Motion_Car_Control(200, 0, -500);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "ROTATE LEFT") == 0 || strcmp(msg, "rotate left") == 0) {
        DEBUG_PRINT("ROTATE LEFT!\r\n");
        Motion_Car_Control(0, 0, 700);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "ROTATE RIGHT") == 0 || strcmp(msg, "rotate right") == 0) {
        DEBUG_PRINT("ROTATE RIGHT!\r\n");
        Motion_Car_Control(0, 0, -700);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else {
        DEBUG_PRINT("Unknown command: %s\n", msg);
    }
}

void Human_Face_Track(int x, int w, int h)
{
    if (!Wait_Flag) {
        delay(1000); delay(1000); delay(1000);
        Wait_Flag = 1;
        return;
    }

    if (Wait_Flag && !SetTarget_Flag) {
        SetTarget_Flag = 1;
        target_area = w * h / 100;
    }

    int now_area = w * h / 100;

    int16_t err = 280 - x;
    DEBUG_PRINT("x:%d, CENTER:%d\r\n", x, 280);
    DEBUG_PRINT("err:%d\r\n", err);

    pid_output = PID_Calc(err);
    DEBUG_PRINT("pid_output:%d\r\n", pid_output);

    DEBUG_PRINT("\r\n");

    int16_t err_area = target_area - now_area;
    DEBUG_PRINT("now_area:%d, target_area:%d\r\n", now_area, target_area);
    DEBUG_PRINT("err_area:%d\r\n", err_area);

    if (err_area < 30 && err_area > -30) err_area = 0;
    pid_output1 = PID_Calc(err_area);
    DEBUG_PRINT("pid_output1:%d\r\n", pid_output1);
    if (pid_output1 > 300)  pid_output1 = 300;
    if (pid_output1 < -300) pid_output1 = -300;

    DEBUG_PRINT("\r\n");

    Motion_Car_Control(pid_output1, 0, pid_output);
}

void AprilTag_Track(int x, int w, int h)
{
    if (!Wait_Flag) {
        delay(1000); delay(1000); delay(1000);
        Wait_Flag = 1;
        return;
    }

    if (Wait_Flag && !SetTarget_Flag) {
        SetTarget_Flag = 1;
        target_area = w * h / 100;
    }

    int now_area = w * h / 100;

    int16_t err = 200 - x;
    DEBUG_PRINT("x:%d, CENTER:%d\r\n", x, 200);
    DEBUG_PRINT("err:%d\r\n", err);

    pid_output = PID_Calc(err);
    DEBUG_PRINT("pid_output:%d\r\n", pid_output);

    DEBUG_PRINT("\r\n");

    int16_t err_area = target_area - now_area;
    DEBUG_PRINT("now_area:%d, target_area:%d\r\n", now_area, target_area);
    DEBUG_PRINT("err_area:%d\r\n", err_area);

    if (err_area < 2 && err_area > -2) err_area = 0;
    pid_output1 = PID_Calc(err_area);
    DEBUG_PRINT("pid_output1:%d\r\n", pid_output1);
    if (pid_output1 > 300)  pid_output1 = 300;
    if (pid_output1 < -300) pid_output1 = -300;

    DEBUG_PRINT("\r\n");

    Motion_Car_Control(pid_output1, 0, pid_output);
}

void Licence_Track(int x1, int y1, int x2, int y2, int x3)
{
    if (!Wait_Flag) {
        delay(1000); delay(1000); delay(1000);
        Wait_Flag = 1;
        return;
    }

    if (Wait_Flag && !SetTarget_Flag) {
        SetTarget_Flag = 1;
        target_area = (x3 - x1) * (y2 - y1) / 100;
    }

    int now_area = (x3 - x1) * (y2 - y1) / 100;

    int16_t err = 240 - x1;
    DEBUG_PRINT("x1:%d, CENTER:%d\r\n", x1, 240);
    DEBUG_PRINT("err:%d\r\n", err);
    if (err < 5 && err > -5) err = 0;

    pid_output = PID_Calc(err);
    DEBUG_PRINT("pid_output:%d\r\n", pid_output);
    if (pid_output > 500)  pid_output = 500;
    if (pid_output < -500) pid_output = -500;

    DEBUG_PRINT("\r\n");

    int16_t err_area = target_area - now_area;
    DEBUG_PRINT("now_area:%d, target_area:%d\r\n", now_area, target_area);
    DEBUG_PRINT("err_area:%d\r\n", err_area);

    if (err_area < 30 && err_area > -30) err_area = 0;
    pid_output1 = PID_Calc(err_area);
    DEBUG_PRINT("pid_output1:%d\r\n", pid_output1);
    if (pid_output1 > 300)  pid_output1 = 300;
    if (pid_output1 < -300) pid_output1 = -300;

    DEBUG_PRINT("\r\n");

    Motion_Car_Control(pid_output1, 0, pid_output);
}

void Autonomous_Avoid(int w, int h)
{
    target_area = 200;

    int now_area = w * h / 100;

    DEBUG_PRINT("now_area:%d, target_area:%d\r\n", now_area, target_area);
    DEBUG_PRINT("\r\n");

    if (now_area >= target_area) {
        Motion_Car_Control(0, 0, 0);
    }
    delay(100);
}

void RoadSign_Rec(const char* msg)
{
    if (strcmp(msg, "go_straight") == 0) {
        DEBUG_PRINT("go straight!\r\n");
        Motion_Car_Control(200, 0, 0);
        delay(1000); delay(1000);
    } else if (strcmp(msg, "turn_right") == 0) {
        DEBUG_PRINT("turn right!\r\n");
        Motion_Car_Control(200, 0, -300);
        delay(1000); delay(1000);
    } else if (strcmp(msg, "speed_limit") == 0) {
        DEBUG_PRINT("speed limit!\r\n");
        Motion_Car_Control(100, 0, 0);
        delay(1000); delay(1000);
    } else if (strcmp(msg, "stop") == 0) {
        DEBUG_PRINT("stop!\r\n");
        Motion_Car_Control(0, 0, 0);
        delay(1000); delay(1000);
    } else {
        DEBUG_PRINT("Unknown Sign: %s\n", msg);
    }
}

void Color_Rec(const char* msg)
{
    if (strcmp(msg, "RED") == 0) {
        DEBUG_PRINT("RED!\r\n");
        RGB_Set_Color(255, 255, 0, 0);
        RGB_Update();
        delay(1000);
    } else if (strcmp(msg, "GREEN") == 0) {
        DEBUG_PRINT("GREEN!\r\n");
        RGB_Set_Color(255, 0, 255, 0);
        RGB_Update();
        delay(1000);
    } else if (strcmp(msg, "BLUE") == 0) {
        DEBUG_PRINT("BLUE!\r\n");
        RGB_Set_Color(255, 0, 0, 255);
        RGB_Update();
        delay(1000);
    } else {
        DEBUG_PRINT("Unknown Color: %s\n", msg);
    }
}

void GazeDire_Track(int x_start, int x_end)
{
    int err = x_start - x_end;
    if (err < 40 && err > -40) {
        err = 0;
        Motion_Car_Control(0, 0, 0);
    }
    DEBUG_PRINT("x_start:%d, x_end:%d\r\n", x_start, x_end);
    DEBUG_PRINT("err:%d\r\n", err);

    bool allow_motion = true;

    if (x_start < 270 && err < 0) {
        DEBUG_PRINT("STOP: x_start=%d < 100, err=%d (no right turn)\r\n", x_start, err);
        allow_motion = false;
    } else if (x_start > 400 && err > 0) {
        DEBUG_PRINT("STOP: x_start=%d > 550, err=%d (no left turn)\r\n", x_start, err);
        allow_motion = false;
    }

    if (allow_motion && (err > 40 || err < -40)) {
        pid_output = PID_Calc(err);

        if (pid_output > 250)  pid_output = 250;
        if (pid_output < -250) pid_output = -250;

        DEBUG_PRINT("pid_output:%d\r\n", pid_output);
        DEBUG_PRINT("\r\n");
        Motion_Car_Control(0, 0, pid_output);
    } else if (!allow_motion) {
        Motion_Car_Control(0, 0, 0);
    }
}

void Hand_Track(int x, int w, int h)
{
    if (!Wait_Flag) {
        delay(1000); delay(1000); delay(1000);
        Wait_Flag = 1;
        return;
    }

    if (Wait_Flag && !SetTarget_Flag) {
        SetTarget_Flag = 1;
        target_area = w * h / 100;
        DEBUG_PRINT("target_area:%d\r\n", target_area);
    }

    int now_area = w * h / 100;

    int16_t err = 240 - x;
    DEBUG_PRINT("x:%d, CENTER:%d\r\n", x, 240);
    DEBUG_PRINT("err:%d\r\n", err);

    pid_output = PID_Calc(err);
    if (pid_output > 500)  pid_output = 500;
    if (pid_output < -500) pid_output = -500;
    DEBUG_PRINT("pid_output:%d\r\n", pid_output);

    DEBUG_PRINT("\r\n");

    int16_t err_area = target_area - now_area;
    DEBUG_PRINT("now_area:%d, target_area:%d\r\n", now_area, target_area);
    DEBUG_PRINT("err_area:%d\r\n", err_area);

    if (err_area < 100 && err_area > -100) err_area = 0;
    pid_output1 = PID_Calc(err_area);
    DEBUG_PRINT("pid_output1:%d\r\n", pid_output1);
    if (pid_output1 > 300)  pid_output1 = 300;
    if (pid_output1 < -300) pid_output1 = -300;

    DEBUG_PRINT("\r\n");

    Motion_Car_Control(pid_output1, 0, pid_output);
}

void Target_Track(int x)
{
    int16_t err = 280 - x;
    DEBUG_PRINT("x:%d, CENTER:%d\r\n", x, 250);
    DEBUG_PRINT("err:%d\r\n", err);
    if (err < 5 && err > -5) err = 0;

    pid_output = PID_Calc(err);
    DEBUG_PRINT("pid_output:%d\r\n", pid_output);
    if (pid_output > 700)  pid_output = 700;
    if (pid_output < -700) pid_output = -700;

    DEBUG_PRINT("\r\n");
    Motion_Car_Control(0, 0, pid_output);
}

void HumanBody_Track(int x, int w, int h)
{
    if (!Wait_Flag) {
        delay(1000);
        Wait_Flag = 1;
        return;
    }

    if (Wait_Flag && !SetTarget_Flag) {
        SetTarget_Flag = 1;
        target_area = w * h / 100;
        DEBUG_PRINT("target_area:%d\r\n", target_area);
    }

    int now_area = w * h / 100;

    int16_t err = 260 - x;
    DEBUG_PRINT("x:%d, CENTER:%d\r\n", x, 260);
    DEBUG_PRINT("err:%d\r\n", err);
    if (err < 5 && err > -5) err = 0;

    pid_output = PID_Calc(err);
    DEBUG_PRINT("pid_output:%d\r\n", pid_output);
    if (pid_output > 400)  pid_output = 400;
    if (pid_output < -400) pid_output = -400;

    DEBUG_PRINT("\r\n");

    int16_t err_area = target_area - now_area;
    DEBUG_PRINT("now_area:%d, target_area:%d\r\n", now_area, target_area);
    DEBUG_PRINT("err_area:%d\r\n", err_area);

    if (err_area < 80 && err_area > -80) err_area = 0;
    pid_output1 = PID_Calc(err_area);
    DEBUG_PRINT("pid_output1:%d\r\n", pid_output1);
    if (pid_output1 > 750)  pid_output1 = 750;
    if (pid_output1 < -750) pid_output1 = -750;

    DEBUG_PRINT("\r\n");

    Motion_Car_Control(pid_output1, 0, pid_output);
}

void OCRrec_Actions(const char* msg)
{
    if (strcmp(msg, "FORWARD") == 0 || strcmp(msg, "forward") == 0) {
        DEBUG_PRINT("FORWARD!\r\n");
        Motion_Car_Control(200, 0, 0);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "BACKWARD") == 0 || strcmp(msg, "backward") == 0) {
        DEBUG_PRINT("BACKWARD!\r\n");
        Motion_Car_Control(-200, 0, 0);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "LEFT") == 0 || strcmp(msg, "left") == 0) {
        DEBUG_PRINT("LEFT!\r\n");
        Motion_Car_Control(200, 0, 300);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "RIGHT") == 0 || strcmp(msg, "right") == 0) {
        DEBUG_PRINT("RIGHT!\r\n");
        Motion_Car_Control(200, 0, -300);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "ROTATELEFT") == 0 || strcmp(msg, "rotateleft") == 0) {
        DEBUG_PRINT("ROTATE LEFT!\r\n");
        Motion_Car_Control(0, 0, 700);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else if (strcmp(msg, "ROTATERIGHT") == 0 || strcmp(msg, "rotateright") == 0) {
        DEBUG_PRINT("ROTATE RIGHT!\r\n");
        Motion_Car_Control(0, 0, -700);
        delay(1000); delay(1000);
        Motion_Car_Control(0, 0, 0);
    } else {
        DEBUG_PRINT("Unknown Character: %s\n", msg);
    }
}
