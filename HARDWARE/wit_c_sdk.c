#include "wit_c_sdk.h"

static SerialWrite p_WitSerialWriteFunc = NULL;
static RegUpdateCb p_WitRegUpdateCbFunc = NULL;
static DelaymsCb   p_WitDelaymsFunc     = NULL;

static uint8_t  s_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
static uint32_t s_uiWitDataCnt = 0, s_uiProtoclo = 0, s_uiReadRegIndex = 0;
int16_t         sReg[REGSIZE];

#define FuncW 0x06
#define FuncR 0x03

static uint8_t __CaliSum(uint8_t *data, uint32_t len) {
    uint32_t i;
    uint8_t  ucCheck = 0;
    for (i = 0; i < len; i++) ucCheck += *(data + i);
    return ucCheck;
}
int32_t WitSerialWriteRegister(SerialWrite Write_func) {
    if (!Write_func) return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}
static void CopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen) {
    uint32_t  uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
    uint16_t *p_usReg1Val = p_data;
    uint16_t *p_usReg2Val = p_data + 3;

    uiReg1Len             = 4;
    switch (ucIndex) {
        case WIT_ACC:
            uiReg1    = AX;
            uiReg1Len = 3;
            uiReg2    = TEMP;
            uiReg2Len = 1;
            break;
        case WIT_ANGLE:
            uiReg1    = Roll;
            uiReg1Len = 3;
            uiReg2    = VERSION;
            uiReg2Len = 1;
            break;
        case WIT_TIME: uiReg1 = YYMM; break;
        case WIT_GYRO:
            uiReg1 = GX;
            uiLen  = 3;
            break;
        case WIT_MAGNETIC:
            uiReg1 = HX;
            uiLen  = 3;
            break;
        case WIT_DPORT:    uiReg1 = D0Status; break;
        case WIT_PRESS:    uiReg1 = PressureL; break;
        case WIT_GPS:      uiReg1 = LonL; break;
        case WIT_VELOCITY: uiReg1 = GPSHeight; break;
        case WIT_QUATER:   uiReg1 = q0; break;
        case WIT_GSA:      uiReg1 = SVNUM; break;
        case WIT_REGVALUE: uiReg1 = s_uiReadRegIndex; break;
        default:           return;
    }
    if (uiLen == 3) {
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
    if (uiReg1Len) {
        memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len << 1);
        p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
    }
    if (uiReg2Len) {
        memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len << 1);
        p_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
    }
}

void WitSerialDataIn(uint8_t ucData) {
    uint16_t /*usCRC16, usTemp, i,*/ usData[4];
    uint8_t                          ucSum;

    if (p_WitRegUpdateCbFunc == NULL) return;
    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;
    switch (s_uiProtoclo) {
        case WIT_PROTOCOL_NORMAL:
            if (s_ucWitDataBuff[0] != 0x55) {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                return;
            }
            if (s_uiWitDataCnt >= 11) {
                ucSum = __CaliSum(s_ucWitDataBuff, 10);
                if (ucSum != s_ucWitDataBuff[10]) {
                    s_uiWitDataCnt--;
                    memcpy(s_ucWitDataBuff, &s_ucWitDataBuff[1], s_uiWitDataCnt);
                    return;
                }
                usData[0] = ((uint16_t)s_ucWitDataBuff[3] << 8) | (uint16_t)s_ucWitDataBuff[2];
                usData[1] = ((uint16_t)s_ucWitDataBuff[5] << 8) | (uint16_t)s_ucWitDataBuff[4];
                usData[2] = ((uint16_t)s_ucWitDataBuff[7] << 8) | (uint16_t)s_ucWitDataBuff[6];
                usData[3] = ((uint16_t)s_ucWitDataBuff[9] << 8) | (uint16_t)s_ucWitDataBuff[8];
                CopeWitData(s_ucWitDataBuff[1], usData, 4);
                s_uiWitDataCnt = 0;
            }
            break;
    }
    if (s_uiWitDataCnt == WIT_DATA_BUFF_SIZE) s_uiWitDataCnt = 0;
}

int32_t WitRegisterCallBack(RegUpdateCb update_func) {
    if (!update_func) return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData) {
    //    uint16_t usCRC;
    uint8_t ucBuff[8];
    if (uiReg >= REGSIZE) return WIT_HAL_INVAL;
    switch (s_uiProtoclo) {
        case WIT_PROTOCOL_NORMAL:
            if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;

        default: return WIT_HAL_INVAL;
    }
    return WIT_HAL_OK;
}
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum) {
    //    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    if ((uiReg + uiReadNum) >= REGSIZE) return WIT_HAL_INVAL;
    switch (s_uiProtoclo) {
        case WIT_PROTOCOL_NORMAL:
            if (uiReadNum > 4) return WIT_HAL_INVAL;
            if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;

        default: return WIT_HAL_INVAL;
    }
    s_uiReadRegIndex = uiReg;

    return WIT_HAL_OK;
}
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr) {
    if (uiProtocol > WIT_PROTOCOL_I2C) return WIT_HAL_INVAL;
    s_uiProtoclo = uiProtocol;
    //    s_ucAddr = ucAddr;
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}

int32_t WitDelayMsRegister(DelaymsCb delayms_func) {
    if (!delayms_func) return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

char CheckRange(short sTemp, short sMin, short sMax) {
    if ((sTemp >= sMin) && (sTemp <= sMax))
        return 1;
    else
        return 0;
}

/*change Band*/
int32_t WitSetUartBaud(int32_t uiBaudIndex) {
    if (!CheckRange(uiBaudIndex, WIT_BAUD_4800, WIT_BAUD_921600)) { return WIT_HAL_INVAL; }
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    //	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
        p_WitDelaymsFunc(1);
    else
        ;
    if (WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

/*change output rate */
int32_t WitSetOutputRate(int32_t uiRate) {
    if (!CheckRange(uiRate, RRATE_02HZ, RRATE_NONE)) { return WIT_HAL_INVAL; }
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    //	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL)
        p_WitDelaymsFunc(1);
    else
        ;
    if (WitWriteReg(RRATE, uiRate) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}
