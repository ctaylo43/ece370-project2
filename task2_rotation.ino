#include <WiFi101.h>
#include <WiFiUdp.h>
#include <math.h>
#include <LSM303.h>
#include <Wire.h>
#include <SPI.h>

// Pin Definitions //
#define MOT_L_A 5
#define MOT_L_B 6
#define MOT_R_A 9
#define MOT_R_B 10
#define IRPIN_L 11
#define IRPIN_R 12

#define NORTH 0
#define SOUTH 180
#define EAST 90
#define WEST 270

#define angle_tolerance 15

float cur_angle;

LSM303 compass;

void setup() {
  Serial.begin(9600);
  pinSetup();
  IMU_setup();
  compass.read();
  cur_angle = compass.heading();
  Serial.print("Inital Current Angle: ");
  Serial.println(cur_angle);
}

void loop() {
  Serial.println("Autocorrecting to north");
  delay(5000);
  rotate(NORTH);
  Serial.println("North reached. Heading south");
  delay(5000); // wait 5 seconds
  rotate(SOUTH);
  Serial.println("South reached. Heading east");
  delay(5000); // wait 5 seconds
  rotate(EAST);
  Serial.println("East reached. Heading west");
  delay(5000); // wait 5 seconds
  rotate(WEST);
  Serial.println("West reached. Heading back north");
  delay(5000); // wait 5 seconds
  rotate(NORTH);
  Serial.println("Rotation loop complete.");
  delay(10000); // wait 10 seconds
}

void pinSetup(){
  pinMode(MOT_L_A, OUTPUT);
  pinMode(MOT_L_B, OUTPUT);
  pinMode(MOT_R_A, OUTPUT);
  pinMode(MOT_R_B, OUTPUT);
  pinMode(IRPIN_L, INPUT);
  pinMode(IRPIN_R, INPUT);
  //attachInterrupt(digitalPinToInterrupt(IRPIN_L), ISR_L, RISING);
  //attachInterrupt((digitalPinToInterrupt(IRPIN_R), ISR_R, RISING);
}

void IMU_setup(){
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){+1254,  +465,   -5355};
  compass.m_max = (LSM303::vector<int16_t>){+1628,  +627,  -5099};
}

void rotate(float des_angle){
  int error = cur_angle - des_angle;
  Serial.print("Initial error: ");
  Serial.println(error);
  while(abs(error) > angle_tolerance){

    //if (error > 0){ // rotate counter-clockwise
  analogWrite(MOT_R_A, 100);
  analogWrite(MOT_R_B, 0);
  analogWrite(MOT_L_A, 0);
  analogWrite(MOT_L_B, 100);
    //}
//    else { // rotate clockwise
//      analogWrite(MOT_R_A, 0);
//      analogWrite(MOT_R_B, 100);
//      analogWrite(MOT_L_A, 100);
//      analogWrite(MOT_L_B, 0);
//    }
    compass.read();
    cur_angle = compass.heading();
    error = cur_angle - des_angle;
    Serial.print("Current Angle: ");
    Serial.println(cur_angle);
    Serial.print("Current Error: ");
    Serial.println(error);
  }
  Serial.println("Rotation complete");
  analogWrite(MOT_R_A, 0);
  analogWrite(MOT_R_B, 0);
  analogWrite(MOT_L_A, 0);
  analogWrite(MOT_L_B, 0);
}
