#include <WiFi101.h>
#include <WiFiUdp.h>
#include <math.h>
#include <LSM303.h>
#include <Wire.h>
#include <SPI.h>
#include "arduino_secreyts.h"

// Pin Definitions //
#define MOT_L_A 5
#define MOT_L_B 6
#define MOT_R_A 9
#define MOT_R_B 10
#define IRPIN_L 11
#define IRPIN_R 12

// WiFi Variables // 
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
IPAddress ip(192,168,1,102); // static ip address
IPAddress gate(192,168,0,1);
IPAddress sub(255,255,0,0);
IPAddress ip2;
IPAddress server(192,168,1,102);

// UDP Variables //
unsigned int localPort = 5005;
WiFiUDP udp;
LSM303 compass;

void setup() {
  APsetup();
  udp.begin(localPort);
  Serial.println("UDP setup complete");
  pinSetup();
  IMU_setup();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void APsetup(){
  // Set WiFi pins on Feather M0
  WiFi.setPins(8,7,4,2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  WiFi.config(ip,gate,sub);
  WiFi.begin(ssid,pass);
  ip2 = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip2);
  Serial.println("Access point setup complete");
}

void pinSetup(){
  pinMode(MOT_L_A, OUTPUT);
  pinMode(MOT_L_B, OUTPUT);
  pinMode(MOT_R_A, OUTPUT);
  pinMode(MOT_R_B, OUTPUT);
  pinMode(IRPIN_L, INPUT);
  pinMode(IRPIN_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRPIN_L), ISR_L, RISING);
  attachInterrupt((digitalPinToInterrupt(IRPIN_R), ISR_R, RISING);
}

void IMU_setup(){
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-5867, -4073, -4321};
  compass.m_max = (LSM303::vector<int16_t>){+935, +2096, +1928};
}
