#include "softtwi.h"
#include "Arduino.h"

#define TIMEOUT 5000 //0.6ms

uint8_t reverse(uint8_t in)
{
  uint8_t out;
  out = 0;
  if (in & 0x01) out |= 0x80;
  if (in & 0x02) out |= 0x40;
  if (in & 0x04) out |= 0x20;
  if (in & 0x08) out |= 0x10;
  if (in & 0x10) out |= 0x08;
  if (in & 0x20) out |= 0x04;
  if (in & 0x40) out |= 0x02;
  if (in & 0x80) out |= 0x01;

  return(out);
}

#define MYDELAY delayMicroseconds(5);

unsigned char i2c_start(unsigned char address)
{
  pinMode(twi_sda, OUTPUT);
  pinMode(twi_scl, OUTPUT);

  digitalWrite(twi_sda, HIGH);MYDELAY
  digitalWrite(twi_scl, HIGH);MYDELAY
  digitalWrite(twi_sda, LOW);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY

  byte b = reverse(address);
  for(int i=0; i<8; i++) {
    if(b&1)
      digitalWrite(twi_sda, HIGH);
    else
      digitalWrite(twi_sda, LOW);

    digitalWrite(twi_scl, HIGH);MYDELAY
    digitalWrite(twi_scl, LOW);MYDELAY
    b = b >> 1;
  }

  // ACK
  digitalWrite(twi_sda, LOW);MYDELAY
  digitalWrite(twi_scl, HIGH);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY

  digitalWrite(twi_sda, LOW);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY

  pinMode(twi_sda, OUTPUT);
  pinMode(twi_scl, OUTPUT);
}

unsigned char i2c_rep_start(unsigned char address)
{
  return i2c_start( address );
}

void i2c_stop(void)
{
  pinMode(twi_sda, OUTPUT);
  pinMode(twi_scl, OUTPUT);

  digitalWrite(twi_sda, LOW);

  digitalWrite(twi_scl, HIGH);
  digitalWrite(twi_sda, HIGH);

  pinMode(twi_sda, INPUT);
  pinMode(twi_scl, INPUT);
}

unsigned char i2c_write( unsigned char data )
{  
  pinMode(twi_sda, OUTPUT);
  pinMode(twi_scl, OUTPUT);

  byte b = reverse(data);
  for(int i=0; i<8; i++) {
    if(b&1)
      digitalWrite(twi_sda, HIGH);
    else
      digitalWrite(twi_sda, LOW);

    digitalWrite(twi_scl, HIGH);MYDELAY
    digitalWrite(twi_scl, LOW);MYDELAY
    b = b >> 1;
  }

  // ACK
  digitalWrite(twi_sda, LOW);MYDELAY
  digitalWrite(twi_scl, HIGH);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY


  digitalWrite(twi_sda, LOW);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY
}

unsigned char i2c_readAck(void)
{
  pinMode(twi_sda, INPUT);
  pinMode(twi_scl, OUTPUT);

  byte b = 0;
  for(int i=0; i<8; i++) {
    if(digitalRead(twi_sda))
      b |= 1;

    digitalWrite(twi_scl, HIGH);MYDELAY
    digitalWrite(twi_scl, LOW);MYDELAY
    if(i!=7)
      b = b << 1;
  }

  pinMode(twi_sda, OUTPUT);

  // ACK
  digitalWrite(twi_sda, LOW);MYDELAY
  digitalWrite(twi_scl, HIGH);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY

  pinMode(twi_sda, OUTPUT);MYDELAY
  pinMode(twi_scl, OUTPUT);MYDELAY

  digitalWrite(twi_sda, LOW);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY

  return b;
}

unsigned char i2c_readNak(void)
{
  pinMode(twi_sda, INPUT);
  pinMode(twi_scl, OUTPUT);

  byte b = 0;
  for(int i=0; i<8; i++) {
    if(digitalRead(twi_sda))
      b |= 1;

    digitalWrite(twi_scl, HIGH);MYDELAY
    digitalWrite(twi_scl, LOW);MYDELAY
    if(i!=7)
      b = b << 1;
  }

  pinMode(twi_sda, OUTPUT);

  // ACK
  digitalWrite(twi_sda, HIGH);MYDELAY
  digitalWrite(twi_scl, HIGH);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY

  pinMode(twi_sda, OUTPUT);MYDELAY
  pinMode(twi_scl, OUTPUT);MYDELAY

  digitalWrite(twi_sda, LOW);MYDELAY
  digitalWrite(twi_scl, LOW);MYDELAY

  return b;
}                                  

