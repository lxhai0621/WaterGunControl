

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
typedef void (*HWTSerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
int32_t HWTWitSerialWriteRegister(HWTSerialWrite write_func);
void HWTWitSerialDataIn(uint8_t ucData);

typedef int32_t (*HWTWitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

typedef int32_t (*HWTWitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
int32_t HWTWitI2cFuncRegister(HWTWitI2cWrite write_func, HWTWitI2cRead read_func);

/* can function */
typedef void (*HWTCanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);
int32_t HWTWitCanWriteRegister(HWTCanWrite write_func);

/* Delayms function */
typedef void (*HWTDelaymsCb)(uint16_t ucMs);
int32_t HWTWitDelayMsRegister(HWTDelaymsCb delayms_func);


void HWTWitCanDataIn(uint8_t ucData[8], uint8_t ucLen);


typedef void (*HWTRegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
int32_t HWTWitRegisterCallBack(HWTRegUpdateCb update_func);
int32_t HWTWitWriteReg(uint32_t uiReg, uint16_t usData);
int32_t HWTWitReadReg(uint32_t uiReg, uint32_t uiReadNum);
int32_t HWTWitInit(uint32_t uiProtocol, uint8_t ucAddr);
void HWTWitDeInit(void);
int32_t HWTWitStartAccCali(void);
int32_t HWTWitStopAccCali(void);
int32_t HWTWitStartMagCali(void);
int32_t HWTWitStopMagCali(void);
int32_t HWTWitSetUartBaud(int32_t uiBaudIndex);
int32_t HWTWitSetBandwidth(int32_t uiBaudWidth);
int32_t HWTWitSetOutputRate(int32_t uiRate);
int32_t HWTWitSetContent(int32_t uiRsw);
int32_t HWTWitSetCanBaud(int32_t uiBaudIndex);

char HWTCheckRange(short sTemp,short sMin,short sMax);

extern int16_t HWTsReg[REGSIZE];

#ifdef __cplusplus
}
#endif
