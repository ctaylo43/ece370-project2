#include <Wire.h>
#include <LSM303.h>
 
LSM303 compass;
 
char report[80];
 
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  Serial.println("setup complete");
}
 
void loop()
{
  Serial.println("loop beginning, going into read()");
  compass.read();
  Serial.println("read complete");
  snprintf(report, sizeof(report), "A: %6d %6d %6d    M: %6d %6d %6d",
    compass.a.x, compass.a.y, compass.a.z,
    compass.m.x, compass.m.y, compass.m.z);
  Serial.println(report);
  Serial.println("looping");
  delay(500);
}
