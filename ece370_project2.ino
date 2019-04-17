#include <time.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <Wire.h>
#include <LSM303.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

#include "arduino_secrets.h"

// Pin Definitions //
#define MOT_L_A 12
#define MOT_L_B 13
#define MOT_R_A 5
#define MOT_R_A 6
#define IRPIN_L 11
#define IRPIN_R 10

// WiFi Variables //
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

// UDP Variables //
#define UDP_PORT = 5000;
char packetBuffer[1023];
WiFiUDP udp;

// Velocity struct
struct Velocity {
  int v;
  int theta;
};
struct Velocity a;

// IMU Variables
LSM303 imu;
bool picked_up;

void setup() {
  APsetup(); // Setup uC access point for pi
  udp.begin(UDP_PORT); // Open ports used for UDP
  initStructs(); // initialize data structures
  IMU_setup(); // set pins for IMU
  //debug_setup(); // serial debugging
}

void loop() {
  checkIMU(); // receive data from IMU
  a = checkUDP(); // receive data structure from UDP
  // (V, theta) = parse(a)
  setSpeed(v); // set speed of robot
  setDir(theta); // set direction of robot
  if (picked_up){ // stop motors when robot is picked up
    mot_R = 0;
    mot_L = 0;
  }
}

// Functions //

void APsetup()
{
  // Set WiFi pins on Feather M0
  WiFi.setPins(8,7,4,2);
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Setting up access point");
  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true);       // don't continue
  }

  Serial.print("SSID: ");
  Serial.println(ssid);
  // Start network
  status = WiFi.beginAP(ssid);
  if (status != WL_AP_LISTENING) {
    Serial.println("Failed to create access point");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
    delay(10000);
    
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void initStructs()
{
  a.v = 0;
  a.theta = 0;
}

void pinSetup()
{
  pinMode(MOT_L_A, OUTPUT);
  pinMode(MOT_L_B, OUTPUT);
  pinMode(MOT_R_A, OUTPUT);
  pinMode(MOT_R_B, OUTPUT);
  pinMode(IRPIN_L, INPUT);
  pinMode(IRPIN_R, INPUT);
  attachInterrupt(IRPIN_L, ISR_L, RISING);
  attachInterrupt(IRPIN_R, ISR_R, RISING);
}

void IMU_setup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
}
