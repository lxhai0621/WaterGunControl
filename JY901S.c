#include "JY901S.h"

static JYSerialWrite JYp_WitSerialWriteFunc = NULL;
static JYWitI2cWrite JYp_WitI2cWriteFunc = NULL;
static JYWitI2cRead JYp_WitI2cReadFunc = NULL;
static JYCanWrite JYp_WitCanWriteFunc = NULL;
static JYRegUpdateCb JYp_WitRegUpdateCbFunc = NULL;
static JYDelaymsCb JYp_WitDelaymsFunc = NULL;

static uint8_t JYs_ucAddr = 0xff;
static uint8_t JYs_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
static uint32_t JYs_uiWitDataCnt = 0, JYs_uiProtoclo = 0, JYs_uiReadRegIndex = 0;
int16_t JYsReg[REGSIZE];


#define FuncW 0x06
#define FuncR 0x03

static const uint8_t JY__auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};
static const uint8_t JY__auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};


static uint16_t JY__CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ JY__auchCRCHi[uIndex];
        uchCRCLo = JY__auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}
static uint8_t JY__CaliSum(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}
int32_t JYWitSerialWriteRegister(JYSerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    JYp_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}
static void JYCopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
    uint16_t *p_usReg1Val = p_data;
    uint16_t *p_usReg2Val = p_data+3;
    
    uiReg1Len = 4;
    switch(ucIndex)
    {
        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  uiReg2 = TEMP;  uiReg2Len = 1;  break;
        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  uiReg2 = VERSION;  uiReg2Len = 1;  break;
        case WIT_TIME:  uiReg1 = YYMM;	break;
        case WIT_GYRO:  uiReg1 = GX;  uiLen = 3;break;
        case WIT_MAGNETIC: uiReg1 = HX;  uiLen = 3;break;
        case WIT_DPORT: uiReg1 = D0Status;  break;
        case WIT_PRESS: uiReg1 = PressureL;  break;
        case WIT_GPS:   uiReg1 = LonL;  break;
        case WIT_VELOCITY: uiReg1 = GPSHeight;  break;
        case WIT_QUATER:    uiReg1 = q0;  break;
        case WIT_GSA:   uiReg1 = SVNUM;  break;
        case WIT_REGVALUE:  uiReg1 = JYs_uiReadRegIndex;  break;
		default:
			return ;

    }
    if(uiLen == 3)
    {
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
    if(uiReg1Len)
	{
		memcpy(&JYsReg[uiReg1], p_usReg1Val, uiReg1Len<<1);
		JYp_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
	}
    if(uiReg2Len)
	{
		memcpy(&JYsReg[uiReg2], p_usReg2Val, uiReg2Len<<1);
		JYp_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
	}
}

void JYWitSerialDataIn(uint8_t ucData)
{
    uint16_t usCRC16, usTemp, i, usData[4];
    uint8_t ucSum;

    if(JYp_WitRegUpdateCbFunc == NULL)return ;
    JYs_ucWitDataBuff[JYs_uiWitDataCnt++] = ucData;
    switch(JYs_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(JYs_ucWitDataBuff[0] != 0x55)
            {
                JYs_uiWitDataCnt--;
                memcpy(JYs_ucWitDataBuff, &JYs_ucWitDataBuff[1], JYs_uiWitDataCnt);
                return ;
            }
            if(JYs_uiWitDataCnt >= 11)
            {
                ucSum = JY__CaliSum(JYs_ucWitDataBuff, 10);
                if(ucSum != JYs_ucWitDataBuff[10])
                {
                    JYs_uiWitDataCnt--;
                    memcpy(JYs_ucWitDataBuff, &JYs_ucWitDataBuff[1], JYs_uiWitDataCnt);
                    return ;
                }
                usData[0] = ((uint16_t)JYs_ucWitDataBuff[3] << 8) | (uint16_t)JYs_ucWitDataBuff[2];
                usData[1] = ((uint16_t)JYs_ucWitDataBuff[5] << 8) | (uint16_t)JYs_ucWitDataBuff[4];
                usData[2] = ((uint16_t)JYs_ucWitDataBuff[7] << 8) | (uint16_t)JYs_ucWitDataBuff[6];
                usData[3] = ((uint16_t)JYs_ucWitDataBuff[9] << 8) | (uint16_t)JYs_ucWitDataBuff[8];
                JYCopeWitData(JYs_ucWitDataBuff[1], usData, 4);
                JYs_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_MODBUS:
            if(JYs_uiWitDataCnt > 2)
            {
                if(JYs_ucWitDataBuff[1] != FuncR)
                {
                    JYs_uiWitDataCnt--;
                    memcpy(JYs_ucWitDataBuff, &JYs_ucWitDataBuff[1], JYs_uiWitDataCnt);
                    return ;
                }
                if(JYs_uiWitDataCnt < (JYs_ucWitDataBuff[2] + 5))return ;
                usTemp = ((uint16_t)JYs_ucWitDataBuff[JYs_uiWitDataCnt-2] << 8) | JYs_ucWitDataBuff[JYs_uiWitDataCnt-1];
                usCRC16 = JY__CRC16(JYs_ucWitDataBuff, JYs_uiWitDataCnt-2);
                if(usTemp != usCRC16)
                {
                    JYs_uiWitDataCnt--;
                    memcpy(JYs_ucWitDataBuff, &JYs_ucWitDataBuff[1], JYs_uiWitDataCnt);
                    return ;
                }
                usTemp = JYs_ucWitDataBuff[2] >> 1;
                for(i = 0; i < usTemp; i++)
                {
                    JYsReg[i+JYs_uiReadRegIndex] = ((uint16_t)JYs_ucWitDataBuff[(i<<1)+3] << 8) | JYs_ucWitDataBuff[(i<<1)+4];
                }
                JYp_WitRegUpdateCbFunc(JYs_uiReadRegIndex, usTemp);
                JYs_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
        JYs_uiWitDataCnt = 0;
        break;
    }
    if(JYs_uiWitDataCnt == WIT_DATA_BUFF_SIZE)JYs_uiWitDataCnt = 0;
}
int32_t JYWitI2cFuncRegister(JYWitI2cWrite write_func, JYWitI2cRead read_func)
{
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    JYp_WitI2cWriteFunc = write_func;
    JYp_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}
int32_t JYWitCanWriteRegister(JYCanWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    JYp_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}
void JYWitCanDataIn(uint8_t ucData[8], uint8_t ucLen)
{
	uint16_t usData[3];
    if(JYp_WitRegUpdateCbFunc == NULL)return ;
    if(ucLen < 8)return ;
    switch(JYs_uiProtoclo)
    {
        case WIT_PROTOCOL_CAN:
            if(ucData[0] != 0x55)return ;
            usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
            usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
            usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
            JYCopeWitData(ucData[1], usData, 3);
            break;
        case WIT_PROTOCOL_NORMAL:
        case WIT_PROTOCOL_MODBUS:
        case WIT_PROTOCOL_I2C:
            break;
    }
}
int32_t JYWitRegisterCallBack(JYRegUpdateCb update_func)
{
    if(!update_func)return WIT_HAL_INVAL;
    JYp_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}
int32_t JYWitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t ucBuff[8];
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;
    switch(JYs_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(JYp_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            JYp_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(JYp_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = JYs_ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xff;
            usCRC = JY__CRC16(ucBuff, 6);
            ucBuff[6] = usCRC >> 8;
            ucBuff[7] = usCRC & 0xff;
            JYp_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(JYp_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            JYp_WitCanWriteFunc(JYs_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(JYp_WitI2cWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xff;
            ucBuff[1] = usData >> 8;
			if(JYp_WitI2cWriteFunc(JYs_ucAddr << 1, uiReg, ucBuff, 2) != 1)
			{
				//printf("i2c write fail\r\n");
			}
        break;
	default: 
            return WIT_HAL_INVAL;        
    }
    return WIT_HAL_OK;
}
int32_t JYWitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;
    switch(JYs_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(uiReadNum > 4)return WIT_HAL_INVAL;
            if(JYp_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            JYp_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(JYp_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if((usTemp + 5) > WIT_DATA_BUFF_SIZE)return WIT_HAL_NOMEM;
            ucBuff[0] = JYs_ucAddr;
            ucBuff[1] = FuncR;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReadNum >> 8;
            ucBuff[5] = uiReadNum & 0xff;
            usTemp = JY__CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xff;
            JYp_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(uiReadNum > 3)return WIT_HAL_INVAL;
            if(JYp_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            JYp_WitCanWriteFunc(JYs_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(JYp_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
            if(JYp_WitI2cReadFunc(JYs_ucAddr << 1, uiReg, JYs_ucWitDataBuff, usTemp) == 1)
            {
                if(JYp_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                for(i = 0; i < uiReadNum; i++)
                {
                    JYsReg[i+uiReg] = ((uint16_t)JYs_ucWitDataBuff[(i<<1)+1] << 8) | JYs_ucWitDataBuff[i<<1];
                }
                JYp_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
			
            break;
		default: 
            return WIT_HAL_INVAL;
    }
    JYs_uiReadRegIndex = uiReg;

    return WIT_HAL_OK;
}
int32_t JYWitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
	if(uiProtocol > WIT_PROTOCOL_I2C)return WIT_HAL_INVAL;
    JYs_uiProtoclo = uiProtocol;
    JYs_ucAddr = ucAddr;
    JYs_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}
void JYWitDeInit(void)
{
    JYp_WitSerialWriteFunc = NULL;
    JYp_WitI2cWriteFunc = NULL;
    JYp_WitI2cReadFunc = NULL;
    JYp_WitCanWriteFunc = NULL;
    JYp_WitRegUpdateCbFunc = NULL;
    JYs_ucAddr = 0xff;
    JYs_uiWitDataCnt = 0;
    JYs_uiProtoclo = 0;
}

int32_t JYWitDelayMsRegister(JYDelaymsCb delayms_func)
{
    if(!delayms_func)return WIT_HAL_INVAL;
    JYp_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

char JYCheckRange(short sTemp,short sMin,short sMax)
{
    if ((sTemp>=sMin)&&(sTemp<=sMax)) return 1;
    else return 0;
}
/*Acceleration calibration demo*/
int32_t JYWitStartAccCali(void)
{
/*
	First place the equipment horizontally, and then perform the following operations
*/
	if(JYWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t JYWitStopAccCali(void)
{
	if(JYWitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*Magnetic field calibration*/
int32_t JYWitStartMagCali(void)
{
	if(JYWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(CALSW, CALMAGMM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t JYWitStopMagCali(void)
{
	if(JYWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Band*/
int32_t JYWitSetUartBaud(int32_t uiBaudIndex)
{
	if(!JYCheckRange(uiBaudIndex,WIT_BAUD_4800,WIT_BAUD_230400))
	{
		return WIT_HAL_INVAL;
	}
	if(JYWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Can Band*/
int32_t JYCWitSetCanBaud(int32_t uiBaudIndex)
{
	if(!JYCheckRange(uiBaudIndex,CAN_BAUD_1000000,CAN_BAUD_3000))
	{
		return WIT_HAL_INVAL;
	}
	if(JYWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Bandwidth*/
int32_t JYWitSetBandwidth(int32_t uiBaudWidth)
{	
	if(!JYCheckRange(uiBaudWidth,BANDWIDTH_256HZ,BANDWIDTH_5HZ))
	{
		return WIT_HAL_INVAL;
	}
	if(JYWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(BANDWIDTH, uiBaudWidth) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/*change output rate */
int32_t JYWitSetOutputRate(int32_t uiRate)
{	
	if(!JYCheckRange(uiRate,RRATE_02HZ,RRATE_NONE))
	{
		return WIT_HAL_INVAL;
	}
	if(JYWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(RRATE, uiRate) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/*change JYWitSetContent */
int32_t JYWitSetContent(int32_t uiRsw)
{	
	if(!JYCheckRange(uiRsw,RSW_TIME,RSW_MASK))
	{
		return WIT_HAL_INVAL;
	}
	if(JYWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(JYs_uiProtoclo == WIT_PROTOCOL_MODBUS)	JYp_WitDelaymsFunc(20);
	else if(JYs_uiProtoclo == WIT_PROTOCOL_NORMAL) JYp_WitDelaymsFunc(1);
	else ;
	if(JYWitWriteReg(RSW, uiRsw) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
