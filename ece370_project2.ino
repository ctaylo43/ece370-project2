#include <time.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <Wire.h>
#include <LSM303.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

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
