//teensy microcontroller firmware for worm-like robot
//author: Martin Eder, Technische Universitaet Muenchen, March 2014

#include <math.h>
#include "pressure.h"
#include "stretch_net.h"

#include <Time.h>

#include "MPU6050_6Axis_MotionApps20.h"

// IMU
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include "PCcom.h"

#define NET_ACTIVE
#define DISPLAY_TEST

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)



#define DEBUG

 MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

bool blinkState = false;

int id = 1;	// IMPORTANT: Set ID here <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



void setup() {

	
	Wire.begin();
	Serial.begin(115200);
	Serial2.begin(115200);
	Serial3.begin(115200);

  startConnection(id);


	// initialize device
	delay(3000);

	// Gyro
	Serial.println("Initializing I2C devices...");
	accelgyro.initialize();

	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  
	// configure LED for output
	pinMode(LED_PIN, OUTPUT);  

}

char reqBuffer[256];

uint16_t desiredA = 0;
uint16_t desiredB = 0;
uint16_t desiredC = 0;

uint16_t presA, presB, presC;
uint16_t lenA, lenB, lenC;

bool led_on = false;



void loop() {

  if(getSetPoints(10, &desiredA, &desiredB, &desiredC))
  {
    set(SLAVE_ADDR_ATTINY1, desiredA, 0);    // Set muscle inner pressure for muscle A
    set(SLAVE_ADDR_ATTINY2, desiredB, 0);   // Set muscle inner pressure for muscle B
    set(SLAVE_ADDR_ATTINY4, desiredC, 0);   // Set muscle inner pressure for muscle C

    if(led_on)
      digitalWrite(LED_PIN, HIGH);
    else
      digitalWrite(LED_PIN, LOW);
    led_on = !led_on;
  }



  getall(SLAVE_ADDR_ATTINY1, &presA, &lenA);    // Stretch and pressure data from muscle A
  getall(SLAVE_ADDR_ATTINY2, &presB, &lenB);    // Stretch and pressure data from muscle B
  getall(SLAVE_ADDR_ATTINY4, &presC, &lenC);    // Stretch and pressure data from muscle C
  sendValues(desiredA, desiredB, desiredC, presA, presB, presC, lenA, lenB, lenC);


	delay(10);
}
