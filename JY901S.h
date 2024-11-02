

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "REG.h"


#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */

#define WIT_DATA_BUFF_SIZE  256

#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1
#define WIT_PROTOCOL_CAN    2
#define WIT_PROTOCOL_I2C    3


/* serial function */
typedef void (*JYSerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
int32_t JYWitSerialWriteRegister(JYSerialWrite write_func);
void JYWitSerialDataIn(uint8_t ucData);

typedef int32_t (*JYWitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

typedef int32_t (*JYWitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
int32_t JYWitI2cFuncRegister(JYWitI2cWrite write_func, JYWitI2cRead read_func);

/* can function */
typedef void (*JYCanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);
int32_t JYWitCanWriteRegister(JYCanWrite write_func);

/* Delayms function */
typedef void (*JYDelaymsCb)(uint16_t ucMs);
int32_t JYWitDelayMsRegister(JYDelaymsCb delayms_func);


void JYWitCanDataIn(uint8_t ucData[8], uint8_t ucLen);


typedef void (*JYRegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
int32_t JYWitRegisterCallBack(JYRegUpdateCb update_func);
int32_t JYWitWriteReg(uint32_t uiReg, uint16_t usData);
int32_t JYWitReadReg(uint32_t uiReg, uint32_t uiReadNum);
int32_t JYWitInit(uint32_t uiProtocol, uint8_t ucAddr);
void JYWitDeInit(void);
int32_t JYWitStartAccCali(void);
int32_t JYWitStopAccCali(void);
int32_t JYWitStartMagCali(void);
int32_t JYWitStopMagCali(void);
int32_t JYWitSetUartBaud(int32_t uiBaudIndex);
int32_t JYWitSetBandwidth(int32_t uiBaudWidth);
int32_t JYWitSetOutputRate(int32_t uiRate);
int32_t JYWitSetContent(int32_t uiRsw);
int32_t JYWitSetCanBaud(int32_t uiBaudIndex);

char JYCheckRange(short sTemp,short sMin,short sMax);

extern int16_t JYsReg[REGSIZE];

#ifdef __cplusplus
}
#endif
