#ifndef _PCcom_H_
#define _PCcom_H_

#include <math.h>
#include <Time.h>
#include "Wire.h"

struct tMessage
{
  uint32_t header;
  uint16_t id;
  uint64_t timestamp;
  
  uint16_t desiredA;
  uint16_t desiredB;
  uint16_t desiredC;

  uint16_t pressureA;
  uint16_t pressureB;
  uint16_t pressureC;

  int16_t lengthA;
  int16_t lengthB;
  int16_t lengthC;
};

struct tRequest
{
  uint32_t header;
  
  uint16_t desiredA;
  uint16_t desiredB;
  uint16_t desiredC;  
};

void startConnection(uint16_t id);

bool sendValues(uint16_t desA, uint16_t desB, uint16_t desC, uint16_t presA, uint16_t presB, uint16_t presC, int16_t lenA, int16_t lenB, int16_t lenC);
bool getSetPoints(int timeout, uint16_t *pSetA, uint16_t *pSetB, uint16_t *pSetC);


#endif // _PCcom_H_
