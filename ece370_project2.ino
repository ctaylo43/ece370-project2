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

// WiFi Variables //
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

void setup() {
  APsetup(); // Setup uC access point for pi
  openPorts(); // Open ports used for UDP
  initStructs(); // initialize data structures
  IR_setup(); // set pins IR sensor
  motor_setup(); // set pins for motor
  IMU_setup(); // set pins for IMU
  debug_setup(); // serial debugging
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

void APsetup(){
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
