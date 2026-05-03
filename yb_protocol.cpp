#include "yb_protocol.h"

#define DEBUG 0
#if DEBUG
    #define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)
#endif

uint8_t RxBuffer[PTO_BUF_LEN_MAX];
uint8_t RxIndex = 0;
uint8_t RxFlag = 0;
uint8_t New_CMD_flag;
uint8_t New_CMD_length;
volatile int lost_count = 50;
volatile int Lost_Flag = 0;

extern int Car_Auto_Drive;

extern ColorMode color_mode;
int x, y, w, h, id, degrees, x0, y0, x1, y1 = 0;
float score = 0;

/////////////////////// Field Descriptions ////////////////////////

static const FieldMeta color_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}
};

static const FieldMeta barcode_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}
};

static const FieldMeta qrcode_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}
};

static const FieldMeta apriltag_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_INT}, {7, FIELD_INT}
};

static const FieldMeta dmcode_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}, {7, FIELD_INT}
};

static const FieldMeta facedetect_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}
};

static const FieldMeta eyegaze_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}
};

static const FieldMeta facerecog_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}, {7, FIELD_INT}
};

static const FieldMeta persondetect_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}
};

static const FieldMeta falldown_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}, {7, FIELD_INT}
};

static const FieldMeta handdetect_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}
};

static const FieldMeta handgesture_fields[] = {
    {2, FIELD_STRING}
};

static const FieldMeta ocrrec_fields[] = {
    {2, FIELD_STRING}
};

static const FieldMeta objectdetect_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}
};

static const FieldMeta nanotracker_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}
};

static const FieldMeta selflearning_fields[] = {
    {2, FIELD_STRING}, {3, FIELD_INT}
};

static const FieldMeta licencerec_fields[] = {
    {2, FIELD_STRING}
};

static const FieldMeta licencedetect_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_INT}, {7, FIELD_INT}, {8, FIELD_INT}, {9, FIELD_INT}
};

static const FieldMeta garbagedetect_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}
};

static const FieldMeta guidedetect_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}
};

static const FieldMeta obstacledetect_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}
};

static const FieldMeta multicolorrec_fields[] = {
    {2, FIELD_INT}, {3, FIELD_INT}, {4, FIELD_INT}, {5, FIELD_INT}, {6, FIELD_STRING}
};

const FuncDesc func_table[] = {
    {ID_COLOR,              color_fields,        4, HandleColor},
    {ID_BARCODE,            barcode_fields,      5, HandleBarcode},
    {ID_QRCODE,             qrcode_fields,       5, HandleQRCode},
    {ID_APRILTAG,           apriltag_fields,     6, HandleAprilTag},
    {ID_DMCODE,             dmcode_fields,       6, HandleDMCode},
    {ID_FACE_DETECT,        facedetect_fields,   4, HandleFaceDetect},
    {ID_EYE_GAZE,           eyegaze_fields,      4, HandleEyeGaze},
    {ID_FACE_RECOGNITION,   facerecog_fields,    6, HandleFaceRecognition},
    {ID_PERSON_DETECT,      persondetect_fields, 4, HandlePersonDetect},
    {ID_FALLDOWN_DETECT,    falldown_fields,     6, HandleFallDown},
    {ID_HAND_DETECT,        handdetect_fields,   4, HandleHandDetect},
    {ID_HAND_GESTURE,       handgesture_fields,  1, HandleHandGesture},
    {ID_OCR_REC,            ocrrec_fields,       1, HandleOCRRec},
    {ID_OBJECT_DETECT,      objectdetect_fields, 5, HandleObjectDetect},
    {ID_NANO_TRACKER,       nanotracker_fields,  4, HandleNanoTracker},
    {ID_SELF_LEARNING,      selflearning_fields, 2, HandleSelfLearning},
    {ID_LICENCE_REC,        licencerec_fields,   1, HandleLicenceRec},
    {ID_LICENCE_DETECT,     licencedetect_fields,8, HandleLicenceDetect},
    {ID_GARBAGE_DETECT,     garbagedetect_fields,5, HandleGarbageDetect},
    {ID_GUIDE_DETECT,       guidedetect_fields,  5, HandleGuideDetect},
    {ID_OBSTACLE_DETECT,    obstacledetect_fields,5, HandleObstacleDetect},
    {ID_MULTI_COLOR,        multicolorrec_fields, 5, HandleMultiColorRec},
};

void Pto_Clear_CMD_Flag(void)
{
    for (uint8_t i = 0; i < PTO_BUF_LEN_MAX; i++) {
        RxBuffer[i] = 0;
    }
    New_CMD_length = 0;
    New_CMD_flag = 0;
}

void Pto_Data_Receive(uint8_t Rx_Temp)
{
    switch (RxFlag) {
    case 0:
        if (Rx_Temp == PTO_HEAD) {
            RxBuffer[0] = PTO_HEAD;
            RxFlag = 1;
            RxIndex = 1;
        }
        break;

    case 1:
        RxBuffer[RxIndex] = Rx_Temp;
        RxIndex++;
        if (Rx_Temp == PTO_TAIL) {
            New_CMD_flag = 1;
            New_CMD_length = RxIndex;
            RxFlag = 0;
            RxIndex = 0;
        } else if (RxIndex >= PTO_BUF_LEN_MAX) {
            New_CMD_flag = 0;
            New_CMD_length = 0;
            RxFlag = 0;
            RxIndex = 0;
            Pto_Clear_CMD_Flag();
        }
        break;

    default:
        break;
    }
}

void Get_LOST_Flag(void)
{
    if (lost_count > 0) {
        Lost_Flag = 0;
        lost_count--;
    }
    if (lost_count == 0) {
        Lost_Flag = 1;
        lost_count = 50;
    }
}

int Pto_Char_To_Int(char* data)
{
    return atoi(data);
}

void Pto_Data_Parse(uint8_t *data_buf, uint8_t num)
{
    uint8_t pto_head = data_buf[0];
    uint8_t pto_tail = data_buf[num - 1];

    if (!(pto_head == PTO_HEAD && pto_tail == PTO_TAIL)) {
        return;
    }

    uint8_t data_index = 1;
    uint8_t field_index[PTO_BUF_LEN_MAX] = {0};
    int i = 0;
    int values[PTO_BUF_LEN_MAX] = {0};
    char msgs[][PTO_BUF_LEN_MAX] = {0};

    for (i = 1; i < num - 1; i++) {
        if (data_buf[i] == ',') {
            data_buf[i] = 0;
            field_index[data_index] = i;
            data_index++;
        }
    }

    for (i = 0; i < 2; i++) {
        values[i] = Pto_Char_To_Int((char*)data_buf + field_index[i] + 1);
    }

    uint8_t pto_len = values[0];
    uint8_t pto_id = values[1];

    ParseCommonFields(pto_id, data_buf, pto_len, field_index, data_index, values, msgs);
}

void ParseCommonFields(
    int func_index,
    uint8_t *data_buf,
    uint8_t num,
    uint8_t *field_index,
    uint8_t data_index,
    int* values,
    char msgs[][PTO_BUF_LEN_MAX])
{
    const FuncDesc* desc = &func_table[func_index - 1];

    if (func_index != desc->id) {
        DEBUG_PRINT("Unsupported protocol ID: %d, Chosen ID:%d\n", func_index, desc->id);
        return;
    }

    for (int i = 0; i < desc->field_count; i++) {
        const FieldMeta* meta = &desc->fields[i];
        uint8_t pos = meta->pos;

        if (meta->type == FIELD_STRING) {
            int start = field_index[pos] + 1;
            int end = (pos + 1 < data_index) ? field_index[pos + 1] : (num - 1);
            int len = end - start;

            len = (len < PTO_BUF_LEN_MAX - 1) ? len : PTO_BUF_LEN_MAX - 1;
            memcpy(msgs[i], data_buf + start, len);
            msgs[i][len] = '\0';
        } else {
            values[pos] = Pto_Char_To_Int((char*)data_buf + field_index[pos] + 1);
        }
    }
    desc->handler(values, msgs);
}

void Pto_Loop(void)
{
    if (New_CMD_flag) {
        Pto_Data_Parse((uint8_t*)RxBuffer, New_CMD_length);
        Pto_Clear_CMD_Flag();
    }

    Get_LOST_Flag();
    if (Car_Auto_Drive && Lost_Flag) {
        Motion_Car_Control(200, 0, 0);
    } else if (Lost_Flag) {
        Motion_Car_Control(0, 0, 0);
        Lost_Flag = 0;
    }
}

/////////////////////////// Protocol Handlers ///////////////////////////

void HandleColor(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    x = values[2]; y = values[3]; w = values[4]; h = values[5];
    DEBUG_PRINT("color:x:%d, y:%d, w:%d, h:%d\n", x, y, w, h);
    switch (color_mode) {
        case COLOR_MODE_FOLLOW_LINE:
            Visual_Line_Track(x);
            break;
        case COLOR_MODE_TRACE:
            Color_Trace(x, w, h);
            break;
    }
}

void HandleBarcode(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    const char* msg = msgs[4];
    DEBUG_PRINT("barcode:x:%d, y:%d, w:%d, h:%d, msg:'%s'\n", values[2], values[3], values[4], values[5], msg);
}

void HandleQRCode(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    const char* msg = msgs[4];
    DEBUG_PRINT("qrcode:x:%d, y:%d, w:%d, h:%d, msg:'%s'\n", values[2], values[3], values[4], values[5], msg);
    QRCode_Action(msg);
}

void HandleAprilTag(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    x = values[2]; y = values[3]; w = values[4]; h = values[5];
    id = values[6]; degrees = values[7];
    DEBUG_PRINT("apriltag:x:%d, y:%d, w:%d, h:%d, id:%d, degrees:%d\n", x, y, w, h, id, degrees);
    AprilTag_Track(x, w, h);
}

void HandleDMCode(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    const char* msg = msgs[4];
    DEBUG_PRINT("dmcode:x:%d, y:%d, w:%d, h:%d, msg:'%s', degrees:%d\n", values[2], values[3], values[4], values[5], msg, values[6]);
}

void HandleFaceDetect(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    x = values[2]; y = values[3]; w = values[4]; h = values[5];
    DEBUG_PRINT("face:x:%d, y:%d, w:%d, h:%d\n", x, y, w, h);
    Human_Face_Track(x, w, h);
}

void HandleEyeGaze(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    x0 = values[2]; y0 = values[3]; x1 = values[4]; y1 = values[5];
    DEBUG_PRINT("eye:x0:%d, y0:%d, x1:%d, y1:%d\n", x0, y0, x1, y1);
    GazeDire_Track(x0, x1);
}

void HandleFaceRecognition(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    const char* name = msgs[4];
    score = values[6] / 100.0;
    DEBUG_PRINT("face recognition:x:%d, y:%d, w:%d, h:%d, name:'%s', score:%.2f\n", values[2], values[3], values[4], values[5], name, score);
}

void HandlePersonDetect(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    x = values[2]; y = values[3]; w = values[4]; h = values[5];
    DEBUG_PRINT("person:x:%d, y:%d, w:%d, h:%d\n", x, y, w, h);
    HumanBody_Track(x, w, h);
}

void HandleFallDown(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    const char* state = msgs[4];
    score = values[6] / 100.0;
    DEBUG_PRINT("falldown:x:%d, y:%d, w:%d, h:%d, state:'%s', score:%.2f\n", values[2], values[3], values[4], values[5], state, score);
}

void HandleHandDetect(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    x = values[2]; y = values[3]; w = values[4]; h = values[5];
    DEBUG_PRINT("hand:x:%d, y:%d, w:%d, h:%d\n", x, y, w, h);
    Hand_Track(x, w, h);
}

void HandleHandGesture(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    DEBUG_PRINT("gesture:'%s'\n", msgs[0]);
}

void HandleOCRRec(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    DEBUG_PRINT("ocr_rec:'%s'\n", msgs[0]);
    OCRrec_Actions(msgs[0]);
}

void HandleObjectDetect(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    DEBUG_PRINT("object:x:%d, y:%d, w:%d, h:%d, class:'%s'\n", values[2], values[3], values[4], values[5], msgs[4]);
}

void HandleNanoTracker(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    x = values[2]; y = values[3]; w = values[4]; h = values[5];
    DEBUG_PRINT("tracker:x:%d, y:%d, w:%d, h:%d\n", x, y, w, h);
    Target_Track(x);
}

void HandleSelfLearning(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    score = values[1] / 100.0;
    DEBUG_PRINT("category:'%s', score:%.2f\n", msgs[0], score);
}

void HandleLicenceRec(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    DEBUG_PRINT("licence_rec:'%s'\n", msgs[0]);
}

void HandleLicenceDetect(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    x1 = values[6]; y1 = values[7];
    int x2 = values[4], y2 = values[5];
    int x3 = values[8], y3 = values[9];
    DEBUG_PRINT("licence_detect:x1:%d, y1:%d, x2:%d, y2:%d, x3:%d, y3:%d\n", x1, y1, x2, y2, x3, y3);
    Licence_Track(x1, y1, x2, y2, x3);
}

void HandleGarbageDetect(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    DEBUG_PRINT("object:x:%d, y:%d, w:%d, h:%d, class:'%s'\n", values[2], values[3], values[4], values[5], msgs[4]);
}

void HandleGuideDetect(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    DEBUG_PRINT("object:x:%d, y:%d, w:%d, h:%d, class:'%s'\n", values[2], values[3], values[4], values[5], msgs[4]);
    RoadSign_Rec(msgs[4]);
}

void HandleObstacleDetect(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    DEBUG_PRINT("object:x:%d, y:%d, w:%d, h:%d, class:'%s'\n", values[2], values[3], values[4], values[5], msgs[4]);
    Autonomous_Avoid(values[4], values[5]);
}

void HandleMultiColorRec(int* values, char msgs[][PTO_BUF_LEN_MAX]) {
    DEBUG_PRINT("object:x:%d, y:%d, w:%d, h:%d, color:'%s'\n", values[2], values[3], values[4], values[5], msgs[4]);
    Color_Rec(msgs[4]);
}
