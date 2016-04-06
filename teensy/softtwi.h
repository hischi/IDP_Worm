#include "Arduino.h"

#define twi_scl  A8
#define twi_sda  A9


unsigned char i2c_read9byte(unsigned char address,uint8_t *_seg1_a1_0, uint8_t *_seg1_a2_0, uint8_t *_seg2_a1_0,uint8_t *_seg2_a2_0,  uint8_t *_seg1_a1_1, uint8_t *_seg1_a2_1, uint8_t *_seg2_a1_1,uint8_t *_seg2_a2_1,uint8_t *bus_b,uint8_t * r);
unsigned char i2c_read1byte(unsigned char address,uint8_t *busb_0);


unsigned char i2c_start(unsigned char address);
unsigned char i2c_rep_start(unsigned char address);
void i2c_stop(void);
unsigned char i2c_write( unsigned char data );
unsigned char i2c_readAck(void);
unsigned char i2c_readNak(void);
