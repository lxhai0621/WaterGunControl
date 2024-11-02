#include <PID_v1.h>
#include "JY901S.h"
#include "robomodule_2515_CAN.h"
#include "CreateHWT101.h"
#include "CreateJY.h"
#include <Wire.h>
double Input1, Output1, Setpoint1;
double Input2, Output2, Setpoint2;
double Kp1 = 4000, Ki1 = 0, Kd1 = 220;
double Kp2 = 5000, Ki2 = 0, Kd2 = 250;
PID myPID1(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, REVERSE);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, REVERSE);
class WaterControl
{
  private:
    HWT101 HWT;
    JY901 JY;
    CRobomodule_2515_CAN ocan;
    double Z_Angle;
    double Y_Angle;
    unsigned long last_time;
  public:
    WaterControl();
    void gyroscope_init();
    void mcp2515_init();
    void pid_init();
    void show_gyroscope_data();
    void set_angle(double Z_Angle, double Y_Angle);
    void water_delay(long times);
};
WaterControl::WaterControl()
{
  Input1 = 0;
  Input2 = 0;
  Setpoint1 = 0;
  Setpoint2 = 0;
  last_time=0;
}
void WaterControl::gyroscope_init()
{
  Serial.begin(115200);
  HWT.HWT101setup();
  JY.JY901setup();
}
void WaterControl::mcp2515_init()
{
  ocan.initdriver(CAN_1000KBPS, 0, 1, 1);
  ocan.initdriver(CAN_1000KBPS, 0, 2, 1);
}
void WaterControl::pid_init()
{
  myPID1.SetOutputLimits(-1800, 1800);
  myPID1.SetSampleTime(10);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-1200, 2500);
  myPID2.SetSampleTime(10);
  myPID2.SetMode(AUTOMATIC);
}
void WaterControl::show_gyroscope_data()
{
  HWT.HWT101loop();
  JY.JY901loop();
  Input1 = HWT.HWTfAngle[2] + 180;
  Input2 = JY.JYfAngle[0] + 160;
  Serial.print("Input1:");
  Serial.print(Input1);
  Serial.print("Output1:");
  Serial.print(Output1);
  Serial.print("Input2:");
  Serial.print(Input2);
  Serial.print("Output2:");
  Serial.print(Output2);
  Serial.print("\n");
}
void WaterControl::set_angle(double Z_Angle, double Y_Angle)
{
  Setpoint1 = Z_Angle;
  Setpoint2 = Y_Angle;
  while (abs(Input1 - Z_Angle) > 0.3 || abs(Input2 - Y_Angle) > 0.3) {
    HWT.HWT101loop();
    JY.JY901loop();
    Input1 = HWT.HWTfAngle[2] + 180;
    Input2 = JY.JYfAngle[0] + 160;
    myPID1.Compute();
    ocan.speedwheel(Output1, 0, 1);
    myPID2.Compute();
    ocan.speedwheel(Output2, 0, 2);
    
  }
  last_time=millis();
  ocan.speedwheel(0, 0, 1);
  ocan.speedwheel(0, 0, 2);
}
void WaterControl::water_delay(long times)
{
  unsigned long currentMillis=millis();
  while(true){
    if(currentMillis-last_time>times){
      break;
    }
   currentMillis = millis(); 
  }
}
