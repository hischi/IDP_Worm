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



//#define DEBUG

 MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount=0;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
int32_t qI[4];

VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int32_t gyro_data[3];
int16_t quat[4];


volatile uint8_t mpuInterrupt = 0;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = 1;
}

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

#ifdef DEBUG
  Serial.println("Initializing I2C devices...");
  #endif
  //mpu.initialize();
  
  
  // verify connection
  #ifdef DEBUG
  Serial.println(F("Testing device connections..."));
  #endif
  
  int out = mpu.testConnection();
  
  #ifdef DEBUG
  Serial.println( out ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));  
  #endif
  
  // load and configure the DMP
  #ifdef DEBUG
  Serial.println(F("Initializing DMP..."));
  #endif
  devStatus = mpu.dmpInitialize();
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      #ifdef DEBUG
      Serial.println(F("Enabling DMP..."));
      #endif
      mpu.setDMPEnabled(true);
      
      
      // enable Arduino interrupt detection
      #ifdef DEBUG
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
      #endif
      pinMode(2, INPUT);
      attachInterrupt(2, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      #ifdef DEBUG
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      #endif
      dmpReady = true;
      
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
      #ifdef DEBUG
      Serial.print(F("DMP packetSize ("));
      Serial.print(packetSize);
      Serial.println(F(")"));
        
      Serial.print(F("init fifoCount ("));
      Serial.print(fifoCount);
      Serial.println(F(")"));
      #endif
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      #ifdef DEBUG
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
      #endif
  }
  
	// configure LED for output
	pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, HIGH);
 

}

char reqBuffer[256];

uint16_t desiredA = 0;
uint16_t desiredB = 0;
uint16_t desiredC = 0;

uint16_t presA, presB, presC;
uint16_t lenA, lenB, lenC;

bool led_on = false;

int state = 0;

void loop() {

  state++;
  if(state == 5)
    state = 0;

  if( getSetPoints(10, &desiredA, &desiredB, &desiredC))
  {
    if(desiredA < 500)
      desiredA = 500;

    if(desiredA > 3000)
      desiredA = 3000;

    if(desiredB < 500)
      desiredB = 500;

    if(desiredB > 3000)
      desiredB = 3000;

    if(desiredC < 500)
      desiredC = 500;

    if(desiredC > 3000)
      desiredC = 3000;
    
    set(SLAVE_ADDR_ATTINY1, desiredA, 0);    // Set muscle inner pressure for muscle A
    set(SLAVE_ADDR_ATTINY2, desiredB, 0);   // Set muscle inner pressure for muscle B
    set(SLAVE_ADDR_ATTINY4, desiredC, 0);   // Set muscle inner pressure for muscle C

    
  }


  //if(state == 3)
  {

    if( mpu.dmpPacketAvailable()) {
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = 0;
      mpuIntStatus = mpu.getIntStatus();
      
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
      
      
      // check for overflow (this should never happen unless our code is too inefficient)
      if (fifoCount == 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          //Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else {
        
          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          //fifoCount -= packetSize;
          
          
           mpu.dmpGetQuaternion(quat, fifoBuffer);
           mpu.resetFIFO();
           fifoCount = mpu.getFIFOCount();

            if(led_on)
              digitalWrite(LED_PIN, HIGH);
            else
              digitalWrite(LED_PIN, LOW);
            led_on = !led_on;
              
      }
    }
    
    getall(SLAVE_ADDR_ATTINY1, &presA, &lenA);    // Stretch and pressure data from muscle A
    getall(SLAVE_ADDR_ATTINY2, &presB, &lenB);    // Stretch and pressure data from muscle B
    getall(SLAVE_ADDR_ATTINY4, &presC, &lenC);    // Stretch and pressure data from muscle C
    sendValues(desiredA, desiredB, desiredC, lenA, lenB, quat[0], quat[1], quat[2], quat[3]);
  }


	delay(1);
}
