#include "REG.h"
#include "HWT101.h"
#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
static volatile char HWTs_cDataUpdate = 0, HWTs_cCmd = 0xff;
const uint32_t HWTc_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
class HWT101 {
  public:
  void HWT101setup() {
    // put your setup code here, to run once:
    Serial1.begin(115200);
    HWTWitInit(WIT_PROTOCOL_NORMAL, 0x50);
    HWTWitSerialWriteRegister(HWTSensorUartSend);
    HWTWitRegisterCallBack(HWTSensorDataUpdata);
    HWTWitDelayMsRegister(HWTDelayms);
    HWTAutoScanSensor();
  }
  int i;
  float HWTfAcc[3], HWTfGyro[3], HWTfAngle[3];
  void HWT101loop() {
    while (Serial1.available()) {
      HWTWitSerialDataIn(Serial1.read());
    }
    HWTCmdProcess();
    if (HWTs_cDataUpdate) {
      for (i = 0; i < 3; i++) {
        HWTfAcc[i] = HWTsReg[AX + i] / 32768.0f * 16.0f;
        HWTfGyro[i] = HWTsReg[GX + i] / 32768.0f * 2000.0f;
        HWTfAngle[i] = HWTsReg[Roll + i] / 32768.0f * 180.0f;
      }
      if (HWTs_cDataUpdate & ACC_UPDATE) {

        HWTs_cDataUpdate &= ~ACC_UPDATE;
      }
      if (HWTs_cDataUpdate & GYRO_UPDATE) {

        HWTs_cDataUpdate &= ~GYRO_UPDATE;
      }
      if (HWTs_cDataUpdate & ANGLE_UPDATE) {

        HWTs_cDataUpdate &= ~ANGLE_UPDATE;
      }
      if (HWTs_cDataUpdate & MAG_UPDATE) {

        HWTs_cDataUpdate &= ~MAG_UPDATE;
      }
      HWTs_cDataUpdate = 0;
    }
  }

  void HWTCopeCmdData(unsigned char ucData) {
    static unsigned char HWTs_ucData[50], HWTs_ucRxCnt = 0;

    HWTs_ucData[HWTs_ucRxCnt++] = ucData;
    if (HWTs_ucRxCnt < 3) return;  //Less than three data returned
    if (HWTs_ucRxCnt >= 50) HWTs_ucRxCnt = 0;
    if (HWTs_ucRxCnt >= 3) {
      if ((HWTs_ucData[1] == '\r') && (HWTs_ucData[2] == '\n')) {
        HWTs_cCmd = HWTs_ucData[0];
        memset(HWTs_ucData, 0, 50);
        HWTs_ucRxCnt = 0;
      } else {
        HWTs_ucData[0] = HWTs_ucData[1];
        HWTs_ucData[1] = HWTs_ucData[2];
        HWTs_ucRxCnt = 2;
      }
    }
  }
  static void HWTShowHelp(void) {

  }

  static void HWTCmdProcess(void) {
    switch (HWTs_cCmd) {
      case 'a':
        if (HWTWitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
        break;
      case 'm':
        if (HWTWitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
        break;
      case 'e':
        if (HWTWitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
        break;
      case 'u':
        if (HWTWitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
        break;
      case 'U':
        if (HWTWitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
        break;
      case 'B':
        if (HWTWitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
        else {
          Serial1.begin(HWTc_uiBaud[WIT_BAUD_115200]);
          Serial.print(" 115200 Baud rate modified successfully\r\n");
        }
        break;
      case 'b':
        if (HWTWitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
        else {
          Serial1.begin(HWTc_uiBaud[WIT_BAUD_9600]);
          Serial.print(" 9600 Baud rate modified successfully\r\n");
        }
        break;
      case 'r':
        if (HWTWitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
        else Serial.print("\r\nSet Baud Success\r\n");
        break;
      case 'R':
        if (HWTWitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
        else Serial.print("\r\nSet Baud Success\r\n");
        break;
      case 'C':
        if (HWTWitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
        break;
      case 'c':
        if (HWTWitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
        break;
      case 'h':
        HWTShowHelp();
        break;
      default: break;
    }
    HWTs_cCmd = 0xff;
  }
  static void HWTSensorUartSend(uint8_t *p_data, uint32_t uiSize) {
    Serial1.write(p_data, uiSize);
    Serial1.flush();
  }
  static void HWTDelayms(uint16_t ucMs) {
    delay(ucMs);
  }
  static void HWTSensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    int i;
    for (i = 0; i < uiRegNum; i++) {
      switch (uiReg) {
        case AZ:
          HWTs_cDataUpdate |= ACC_UPDATE;
          break;
        case GZ:
          HWTs_cDataUpdate |= GYRO_UPDATE;
          break;
        case HZ:
          HWTs_cDataUpdate |= MAG_UPDATE;
          break;
        case Yaw:
          HWTs_cDataUpdate |= ANGLE_UPDATE;
          break;
        default:
          HWTs_cDataUpdate |= READ_UPDATE;
          break;
      }
      uiReg++;
    }
  }

  static void HWTAutoScanSensor(void) {
    int i, iRetry;

    for (i = 0; i < sizeof(HWTc_uiBaud) / sizeof(HWTc_uiBaud[0]); i++) {
      Serial1.begin(HWTc_uiBaud[i]);
      Serial1.flush();
      iRetry = 2;
      HWTs_cDataUpdate = 0;
      do {
        int error = HWTWitReadReg(AX, 3);
        Serial.println(error);
        delay(200);
        while (Serial1.available()) {
          HWTWitSerialDataIn(Serial1.read());
        }
        if (HWTs_cDataUpdate != 0) {
          Serial.print(HWTc_uiBaud[i]);
          Serial.print(" baud find sensor\r\n\r\n");
          HWTShowHelp();
          return;
        }
        iRetry--;
      } while (iRetry);
    }
    Serial.print("can not find sensor\r\n");
    Serial.print("please check your connection\r\n");
  }
};
