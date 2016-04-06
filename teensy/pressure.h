//pressure controller board firmware for worm-like robot
//author: Martin Eder, Technische Universitaet Muenchen, March 2014

#ifndef PRESSURE_H
#define PRESSURE_H

#include "softtwi.h"

#define SLAVE_ADDR_ATTINY1 0b00110100
#define SLAVE_ADDR_ATTINY2 0b01110100
#define SLAVE_ADDR_ATTINY3 0b00010100
#define SLAVE_ADDR_ATTINY4 0b00100100

#define SLAVE_ADDR_ATTINY1_EXTRA 0b00111100
#define SLAVE_ADDR_ATTINY2_EXTRA 0b01111100
#define SLAVE_ADDR_ATTINY3_EXTRA 0b00011100
#define SLAVE_ADDR_ATTINY4_EXTRA 0b00101100

#define I2C_READ    1
#define I2C_WRITE   0

void set(unsigned char address, int value, int force) {
  i2c_start(address | I2C_WRITE);
  i2c_write(0);

  i2c_write((value>>8));
  i2c_write(value&0xFF);
  i2c_write((force>>8));
  i2c_write(force&0xFF);
  i2c_write(1);
  i2c_write(2);
  i2c_stop();
}

signed int get(unsigned char address) {
  i2c_start(address | I2C_WRITE);
  i2c_write(0);
  i2c_rep_start(address | I2C_READ);

  int a = (int)i2c_readAck(); // stretch
  int b = (int)i2c_readAck(); // stretch
  int c = (int)i2c_readAck(); // pressure
  int d = (int)i2c_readAck(); // pressure
  int e = (int)i2c_readAck(); // out
  int f = (int)i2c_readNak(); // in

  i2c_stop();
  // pressure return
  //return (c<<8)|d;
  
  //Serial.print((c<<8)|d);
  //Serial.print(", ");
  
  //valve = (c<<8)|d;
  // stretch return
  return (a<<8)|b;
}

signed int getall(unsigned char address, uint16_t *pressure, uint16_t *stretch) {
  i2c_start(address | I2C_WRITE);
  i2c_write(0);
  i2c_rep_start(address | I2C_READ);

  int a = (int)i2c_readAck(); // stretch
  int b = (int)i2c_readAck(); // stretch
  int c = (int)i2c_readAck(); // pressure
  int d = (int)i2c_readAck(); // pressure
  int e = (int)i2c_readAck(); // out
  int f = (int)i2c_readNak(); // in

  i2c_stop();
  
  *pressure = (c<<8)|d;
  *stretch = (a<<8)|b;
}



#endif // PRESSURE_H
