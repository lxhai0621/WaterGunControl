#include "REG.h"
#include "JY901S.h"

/*
Test on MEGA 2560. use WT901CTTL sensor

WT901CTTL     MEGA 2560
    VCC <--->  5V/3.3V
    TX  <--->  19(TX1)
    RX  <--->  18(RX1)
    GND <--->  GND
*/

#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
static volatile char JYs_cDataUpdate = 0, JYs_cCmd = 0xff;
const uint32_t JYc_uiBaud[8] = { 0, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
class JY901 {
  public:
  void JY901setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    JYWitInit(WIT_PROTOCOL_NORMAL, 0x50);
    JYWitSerialWriteRegister(JYSensorUartSend);
    JYWitRegisterCallBack(JYSensorDataUpdata);
    JYWitDelayMsRegister(JYDelayms);
    JYAutoScanSensor();
  }
  int i;
  float JYfAcc[3], JYfGyro[3], JYfAngle[3];
  void JY901loop() {
    while (Serial2.available()) {
      JYWitSerialDataIn(Serial2.read());
    }
    JYCmdProcess();
    if (JYs_cDataUpdate) {
      for (i = 0; i < 3; i++) {
        JYfAcc[i] = JYsReg[AX + i] / 32768.0f * 16.0f;
        JYfGyro[i] = JYsReg[GX + i] / 32768.0f * 2000.0f;
        JYfAngle[i] = JYsReg[Roll + i] / 32768.0f * 180.0f;
        /*
        Serial.print("JYfAcc[0]:");
        Serial.println(JYfAcc[0]);
        Serial.print("JYfAcc[1]:");
        Serial.println(JYfAcc[1]);
        Serial.print("JYfAcc[2]:");
        Serial.println(JYfAcc[2]);
        */
      }
      if (JYs_cDataUpdate & ACC_UPDATE) {

        JYs_cDataUpdate &= ~ACC_UPDATE;
      }
      if (JYs_cDataUpdate & GYRO_UPDATE) {

        JYs_cDataUpdate &= ~GYRO_UPDATE;
      }
      if (JYs_cDataUpdate & ANGLE_UPDATE) {

        JYs_cDataUpdate &= ~ANGLE_UPDATE;
      }
      if (JYs_cDataUpdate & MAG_UPDATE) {
        JYs_cDataUpdate &= ~MAG_UPDATE;
      }
      JYs_cDataUpdate = 0;
    }
  }


  void JYCopeCmdData(unsigned char ucData) {
    static unsigned char JYs_ucData[50], JYs_ucRxCnt = 0;

    JYs_ucData[JYs_ucRxCnt++] = ucData;
    if (JYs_ucRxCnt < 3) return;  //Less than three data returned
    if (JYs_ucRxCnt >= 50) JYs_ucRxCnt = 0;
    if (JYs_ucRxCnt >= 3) {
      if ((JYs_ucData[1] == '\r') && (JYs_ucData[2] == '\n')) {
        JYs_cCmd = JYs_ucData[0];
        memset(JYs_ucData, 0, 50);
        JYs_ucRxCnt = 0;
      } else {
        JYs_ucData[0] = JYs_ucData[1];
        JYs_ucData[1] = JYs_ucData[2];
        JYs_ucRxCnt = 2;
      }
    }
  }
  static void JYShowHelp(void) {
  }

  static void JYCmdProcess(void) {
    switch (JYs_cCmd) {
      case 'a':
        if (JYWitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
        break;
      case 'm':
        if (JYWitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
        break;
      case 'e':
        if (JYWitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
        break;
      case 'u':
        if (JYWitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
        break;
      case 'U':
        if (JYWitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
        break;
      case 'B':
        if (JYWitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
        else {
          Serial2.begin(JYc_uiBaud[WIT_BAUD_115200]);
          Serial.print(" 115200 Baud rate modified successfully\r\n");
        }
        break;
      case 'b':
        if (JYWitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
        else {
          Serial2.begin(JYc_uiBaud[WIT_BAUD_9600]);
          Serial.print(" 9600 Baud rate modified successfully\r\n");
        }
        break;
      case 'r':
        if (JYWitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
        else Serial.print("\r\nSet Baud Success\r\n");
        break;
      case 'R':
        if (JYWitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
        else Serial.print("\r\nSet Baud Success\r\n");
        break;
      case 'C':
        if (JYWitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE | RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
        break;
      case 'c':
        if (JYWitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
        break;
      case 'h':
        JYShowHelp();
        break;
      default: break;
    }
    JYs_cCmd = 0xff;
  }
  static void JYSensorUartSend(uint8_t *p_data, uint32_t uiSize) {
    Serial2.write(p_data, uiSize);
    Serial2.flush();
  }
  static void JYDelayms(uint16_t ucMs) {
    delay(ucMs);
  }
  static void JYSensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    int i;
    for (i = 0; i < uiRegNum; i++) {
      switch (uiReg) {
        case AZ:
          JYs_cDataUpdate |= ACC_UPDATE;
          break;
        case GZ:
          JYs_cDataUpdate |= GYRO_UPDATE;
          break;
        case HZ:
          JYs_cDataUpdate |= MAG_UPDATE;
          break;
        case Yaw:
          JYs_cDataUpdate |= ANGLE_UPDATE;
          break;
        default:
          JYs_cDataUpdate |= READ_UPDATE;
          break;
      }
      uiReg++;
    }
  }

  static void JYAutoScanSensor(void) {
    int i, iRetry;

    for (i = 0; i < sizeof(JYc_uiBaud) / sizeof(JYc_uiBaud[0]); i++) {
      Serial2.begin(JYc_uiBaud[i]);
      Serial2.flush();
      iRetry = 2;
      JYs_cDataUpdate = 0;
      do {
        JYWitReadReg(AX, 3);
        delay(200);
        while (Serial2.available()) {
          JYWitSerialDataIn(Serial2.read());
        }
        if (JYs_cDataUpdate != 0) {
          Serial.print(JYc_uiBaud[i]);
          Serial.print(" baud find sensor\r\n\r\n");
          JYShowHelp();
          return;
        }
        iRetry--;
      } while (iRetry);
    }
    Serial.print("can not find sensor\r\n");
    Serial.print("please check your connection\r\n");
  }
};
