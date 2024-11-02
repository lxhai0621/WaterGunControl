#include "HWT101.h"
#include<stdlib.h>
#include<stdio.h>
static HWTSerialWrite HWTp_WitSerialWriteFunc = NULL;
static HWTWitI2cWrite HWTp_WitI2cWriteFunc = NULL;
static HWTWitI2cRead HWTp_WitI2cReadFunc = NULL;
static HWTCanWrite HWTp_WitCanWriteFunc = NULL;
static HWTRegUpdateCb HWTp_WitRegUpdateCbFunc = NULL;
static HWTDelaymsCb HWTp_WitDelaymsFunc = NULL;

static uint8_t HWTs_ucAddr = 0xff;
static uint8_t HWTs_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
static uint32_t HWTs_uiWitDataCnt = 0, HWTs_uiProtoclo = 0, HWTs_uiReadRegIndex = 0;
int16_t HWTsReg[REGSIZE];


#define FuncW 0x06
#define FuncR 0x03

static const uint8_t HWT__auchCRCHi[256] = {
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
static const uint8_t HWT__auchCRCLo[256] = {
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


static uint16_t HWT__CRC16(uint8_t *puchMsg, uint16_t usDataLen)
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
        uchCRCHi = uchCRCLo ^ HWT__auchCRCHi[uIndex];
        uchCRCLo = HWT__auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}
static uint8_t HWT__CaliSum(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}
int32_t HWTWitSerialWriteRegister(HWTSerialWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    HWTp_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}
static void HWTCopeWitData(uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
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
        case WIT_REGVALUE:  uiReg1 = HWTs_uiReadRegIndex;  break;
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
		memcpy(&HWTsReg[uiReg1], p_usReg1Val, uiReg1Len<<1);
		HWTp_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
	}
    if(uiReg2Len)
	{
		memcpy(&HWTsReg[uiReg2], p_usReg2Val, uiReg2Len<<1);
		HWTp_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
	}
}

void HWTWitSerialDataIn(uint8_t ucData)
{
    uint16_t usCRC16, usTemp, i, usData[4];
    uint8_t ucSum;

    if(HWTp_WitRegUpdateCbFunc == NULL)return ;
    HWTs_ucWitDataBuff[HWTs_uiWitDataCnt++] = ucData;
    switch(HWTs_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(HWTs_ucWitDataBuff[0] != 0x55)
            {
                HWTs_uiWitDataCnt--;
                memcpy(HWTs_ucWitDataBuff, &HWTs_ucWitDataBuff[1], HWTs_uiWitDataCnt);
                return ;
            }
            if(HWTs_uiWitDataCnt >= 11)
            {
                ucSum = HWT__CaliSum(HWTs_ucWitDataBuff, 10);
                if(ucSum != HWTs_ucWitDataBuff[10])
                {
                    HWTs_uiWitDataCnt--;
                    memcpy(HWTs_ucWitDataBuff, &HWTs_ucWitDataBuff[1], HWTs_uiWitDataCnt);
                    return ;
                }
                usData[0] = ((uint16_t)HWTs_ucWitDataBuff[3] << 8) | (uint16_t)HWTs_ucWitDataBuff[2];
                usData[1] = ((uint16_t)HWTs_ucWitDataBuff[5] << 8) | (uint16_t)HWTs_ucWitDataBuff[4];
                usData[2] = ((uint16_t)HWTs_ucWitDataBuff[7] << 8) | (uint16_t)HWTs_ucWitDataBuff[6];
                usData[3] = ((uint16_t)HWTs_ucWitDataBuff[9] << 8) | (uint16_t)HWTs_ucWitDataBuff[8];
                HWTCopeWitData(HWTs_ucWitDataBuff[1], usData, 4);
                HWTs_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_MODBUS:
            if(HWTs_uiWitDataCnt > 2)
            {
                if(HWTs_ucWitDataBuff[1] != FuncR)
                {
                    HWTs_uiWitDataCnt--;
                    memcpy(HWTs_ucWitDataBuff, &HWTs_ucWitDataBuff[1], HWTs_uiWitDataCnt);
                    return ;
                }
                if(HWTs_uiWitDataCnt < (HWTs_ucWitDataBuff[2] + 5))return ;
                usTemp = ((uint16_t)HWTs_ucWitDataBuff[HWTs_uiWitDataCnt-2] << 8) | HWTs_ucWitDataBuff[HWTs_uiWitDataCnt-1];
                usCRC16 = HWT__CRC16(HWTs_ucWitDataBuff, HWTs_uiWitDataCnt-2);
                if(usTemp != usCRC16)
                {
                    HWTs_uiWitDataCnt--;
                    memcpy(HWTs_ucWitDataBuff, &HWTs_ucWitDataBuff[1], HWTs_uiWitDataCnt);
                    return ;
                }
                usTemp = HWTs_ucWitDataBuff[2] >> 1;
                for(i = 0; i < usTemp; i++)
                {
                    HWTsReg[i+HWTs_uiReadRegIndex] = ((uint16_t)HWTs_ucWitDataBuff[(i<<1)+3] << 8) | HWTs_ucWitDataBuff[(i<<1)+4];
                }
                HWTp_WitRegUpdateCbFunc(HWTs_uiReadRegIndex, usTemp);
                HWTs_uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
        HWTs_uiWitDataCnt = 0;
        break;
    }
    if(HWTs_uiWitDataCnt == WIT_DATA_BUFF_SIZE)HWTs_uiWitDataCnt = 0;
}
int32_t HWTWitI2cFuncRegister(HWTWitI2cWrite write_func, HWTWitI2cRead read_func)
{
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    HWTp_WitI2cWriteFunc = write_func;
    HWTp_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}
int32_t HWTWitCanWriteRegister(HWTCanWrite Write_func)
{
    if(!Write_func)return WIT_HAL_INVAL;
    HWTp_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}
void HWTWitCanDataIn(uint8_t ucData[8], uint8_t ucLen)
{
	uint16_t usData[3];
    if(HWTp_WitRegUpdateCbFunc == NULL)return ;
    if(ucLen < 8)return ;
    switch(HWTs_uiProtoclo)
    {
        case WIT_PROTOCOL_CAN:
            if(ucData[0] != 0x55)return ;
            usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
            usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
            usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
            HWTCopeWitData(ucData[1], usData, 3);
            break;
        case WIT_PROTOCOL_NORMAL:
        case WIT_PROTOCOL_MODBUS:
        case WIT_PROTOCOL_I2C:
            break;
    }
}
int32_t HWTWitRegisterCallBack(HWTRegUpdateCb update_func)
{
    if(!update_func)return WIT_HAL_INVAL;
    HWTp_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}
int32_t HWTWitWriteReg(uint32_t uiReg, uint16_t usData)
{
    uint16_t usCRC;
    uint8_t ucBuff[8];
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;
    switch(HWTs_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(HWTp_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            HWTp_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(HWTp_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = HWTs_ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xff;
            usCRC = HWT__CRC16(ucBuff, 6);
            ucBuff[6] = usCRC >> 8;
            ucBuff[7] = usCRC & 0xff;
            HWTp_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(HWTp_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            HWTp_WitCanWriteFunc(HWTs_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(HWTp_WitI2cWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xff;
            ucBuff[1] = usData >> 8;
			if(HWTp_WitI2cWriteFunc(HWTs_ucAddr << 1, uiReg, ucBuff, 2) != 1)
			{
				//printf("i2c write fail\r\n");
			}
        break;
	default: 
            return WIT_HAL_INVAL;        
    }
    return WIT_HAL_OK;
}
int32_t HWTWitReadReg(uint32_t uiReg, uint32_t uiReadNum)
{
    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;
    switch(HWTs_uiProtoclo)
    {
        case WIT_PROTOCOL_NORMAL:
            if(uiReadNum > 4)return WIT_HAL_INVAL;
            if(HWTp_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            HWTp_WitSerialWriteFunc(ucBuff, 5);
            break;
        case WIT_PROTOCOL_MODBUS:
            if(HWTp_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if((usTemp + 5) > WIT_DATA_BUFF_SIZE)return WIT_HAL_NOMEM;
            ucBuff[0] = HWTs_ucAddr;
            ucBuff[1] = FuncR;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReadNum >> 8;
            ucBuff[5] = uiReadNum & 0xff;
            usTemp = HWT__CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xff;
            HWTp_WitSerialWriteFunc(ucBuff, 8);
            break;
        case WIT_PROTOCOL_CAN:
            if(uiReadNum > 3)return WIT_HAL_INVAL;
            if(HWTp_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xff;
            ucBuff[4] = uiReg >> 8;
            HWTp_WitCanWriteFunc(HWTs_ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(HWTp_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
            if(HWTp_WitI2cReadFunc(HWTs_ucAddr << 1, uiReg, HWTs_ucWitDataBuff, usTemp) == 1)
            {
                if(HWTp_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                for(i = 0; i < uiReadNum; i++)
                {
                    HWTsReg[i+uiReg] = ((uint16_t)HWTs_ucWitDataBuff[(i<<1)+1] << 8) | HWTs_ucWitDataBuff[i<<1];
                }
                HWTp_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
			
            break;
		default: 
            return WIT_HAL_INVAL;
    }
    HWTs_uiReadRegIndex = uiReg;

    return WIT_HAL_OK;
}
int32_t HWTWitInit(uint32_t uiProtocol, uint8_t ucAddr)
{
	if(uiProtocol > WIT_PROTOCOL_I2C)return WIT_HAL_INVAL;
    HWTs_uiProtoclo = uiProtocol;
    HWTs_ucAddr = ucAddr;
    HWTs_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}
void HWTWitDeInit(void)
{
    HWTp_WitSerialWriteFunc = NULL;
    HWTp_WitI2cWriteFunc = NULL;
    HWTp_WitI2cReadFunc = NULL;
    HWTp_WitCanWriteFunc = NULL;
    HWTp_WitRegUpdateCbFunc = NULL;
    HWTs_ucAddr = 0xff;
    HWTs_uiWitDataCnt = 0;
    HWTs_uiProtoclo = 0;
}

int32_t HWTWitDelayMsRegister(HWTDelaymsCb delayms_func)
{
    if(!delayms_func)return WIT_HAL_INVAL;
    HWTp_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

char HWTCheckRange(short sTemp,short sMin,short sMax)
{
    if ((sTemp>=sMin)&&(sTemp<=sMax)) return 1;
    else return 0;
}
/*Acceleration calibration demo*/
int32_t HWTWitStartAccCali(void)
{
/*
	First place the equipment horizontally, and then perform the following operations
*/
	if(HWTWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t HWTWitStopAccCali(void)
{
	if(HWTWitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*Magnetic field calibration*/
int32_t HWTWitStartMagCali(void)
{
	if(HWTWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(CALSW, CALMAGMM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
int32_t HWTWitStopMagCali(void)
{
	if(HWTWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Band*/
int32_t HWTWitSetUartBaud(int32_t uiBaudIndex)
{
	if(!HWTCheckRange(uiBaudIndex,WIT_BAUD_4800,WIT_BAUD_230400))
	{
		return WIT_HAL_INVAL;
	}
	if(HWTWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Can Band*/
int32_t HWTCWitSetCanBaud(int32_t uiBaudIndex)
{
	if(!HWTCheckRange(uiBaudIndex,CAN_BAUD_1000000,CAN_BAUD_3000))
	{
		return WIT_HAL_INVAL;
	}
	if(HWTWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
/*change Bandwidth*/
int32_t HWTWitSetBandwidth(int32_t uiBaudWidth)
{	
	if(!HWTCheckRange(uiBaudWidth,BANDWIDTH_256HZ,BANDWIDTH_5HZ))
	{
		return WIT_HAL_INVAL;
	}
	if(HWTWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(BANDWIDTH, uiBaudWidth) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/*change output rate */
int32_t HWTWitSetOutputRate(int32_t uiRate)
{	
	if(!HWTCheckRange(uiRate,RRATE_02HZ,RRATE_NONE))
	{
		return WIT_HAL_INVAL;
	}
	if(HWTWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(RRATE, uiRate) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/*change HWTWitSetContent */
int32_t HWTWitSetContent(int32_t uiRsw)
{	
	if(!HWTCheckRange(uiRsw,RSW_TIME,RSW_MASK))
	{
		return WIT_HAL_INVAL;
	}
	if(HWTWitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(HWTs_uiProtoclo == WIT_PROTOCOL_MODBUS)	HWTp_WitDelaymsFunc(20);
	else if(HWTs_uiProtoclo == WIT_PROTOCOL_NORMAL) HWTp_WitDelaymsFunc(1);
	else ;
	if(HWTWitWriteReg(RSW, uiRsw) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
