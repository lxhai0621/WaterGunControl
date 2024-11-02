#include"angle_control.h"
WaterControl Wctrl;
void setup() 
{
  Wctrl.gyroscope_init();
  Wctrl.mcp2515_init();
  Wctrl.pid_init();
}
int x=25;
void loop() 
{
  /*Wctrl.set_angle(117.9+x,39.32);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(95.16+x,55.37);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(73.79+x,60.74);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(68.13+x,29.94);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(54.9+x,63.83);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(49.37+x,55.57);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(38.36+x,64.9);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(24.41+x,62.45);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(24.41+x,62.45);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(38.36+x,64.9);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(49.37+x,55.57);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(54.9+x,63.83);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(68.13+x,29.94);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(73.79+x,60.74);
  Wctrl.water_delay(5000);
  Wctrl.set_angle(95.16+x,55.37);
  Wctrl.water_delay(5000);*/
  Wctrl.show_gyroscope_data();
}
