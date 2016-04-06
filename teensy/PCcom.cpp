#include "PCcom.h"

#define HEADER 0x7A557E7E

struct tMessage msg;
struct tRequest req;
unsigned long starttime;

#define BUFFER_LEN 256
uint8_t cBuffer[BUFFER_LEN];
int inIdx;
int bufLen;

void startConnection(uint16_t id)
{
  starttime = millis();
  
  msg.header = HEADER;
  msg.id = id;

  inIdx = 0;
  bufLen = 0;
}

bool sendValues(uint16_t desA, uint16_t desB, uint16_t desC, uint16_t presA, uint16_t presB, uint16_t presC, uint16_t lenA, uint16_t lenB, uint16_t lenC)
{
  msg.timestamp = millis()-starttime;
  msg.desiredA = desA;
  msg.desiredB = desB;
  msg.desiredC = desC;

  msg.pressureA = presA;
  msg.lengthA = lenA;

  msg.pressureB = presB;
  msg.lengthB = lenB;

  msg.pressureC = presC;
  msg.lengthC = lenC;

  Serial.write((uint8_t*) &msg, sizeof(msg));
  Serial.flush();

  return true;
}


bool readChar2Buffer()
{
  if(Serial.available() > 0)
  {
    cBuffer[inIdx] = Serial.read();
    inIdx = (inIdx+1)%BUFFER_LEN;
    bufLen++;
    return true;
  }
  return false;
}

bool checkForHeader()
{
  int startIdx = inIdx - bufLen;
  if(startIdx < 0)
    startIdx += BUFFER_LEN;
    
  if(startIdx <= BUFFER_LEN-sizeof(req.header))
    return ((struct tRequest*) &(cBuffer[startIdx]))->header == HEADER;
  else
  {
    uint8_t buf[sizeof(req.header)];
    for(int i = 0; i < sizeof(req.header); i++)
      buf[i] = cBuffer[(startIdx + i)%BUFFER_LEN];

    return ((struct tRequest*) buf)->header == HEADER;
  }
}

void getValuesFromRequest(uint16_t *pSetA, uint16_t *pSetB, uint16_t *pSetC)
{
  int startIdx = inIdx - bufLen;
  if(startIdx < 0)
    startIdx += BUFFER_LEN;
    
  if(startIdx <= BUFFER_LEN-sizeof(tRequest))
  {
    *pSetA = ((struct tRequest*) &(cBuffer[startIdx]))->desiredA;
    *pSetB = ((struct tRequest*) &(cBuffer[startIdx]))->desiredB;
    *pSetC = ((struct tRequest*) &(cBuffer[startIdx]))->desiredC;
  }
  else
  {
    uint8_t buf[sizeof(tRequest)];
    for(int i = 0; i < sizeof(tRequest); i++)
      buf[i] = cBuffer[(startIdx + i)%BUFFER_LEN];

    *pSetA = ((struct tRequest*) &(buf[startIdx]))->desiredA;
    *pSetB = ((struct tRequest*) &(buf[startIdx]))->desiredB;
    *pSetC = ((struct tRequest*) &(buf[startIdx]))->desiredC;
  }
}

#define CHECKTO if(timeout-- <= 0){ goto timeout; }

bool getSetPoints(int timeout, uint16_t *pSetA, uint16_t *pSetB, uint16_t *pSetC)
{
  // Be sure that there are at least 10 bytes in the buffer, otherwise try to get some chars
  while(bufLen < 10)
  {
    CHECKTO;
    readChar2Buffer();
  }

  // As long as we can not find any header, we have skip the first byte and load another one if we have less then 10 bytes
  while(!checkForHeader())
  {
    CHECKTO;
    
    bufLen--;
    if(bufLen < 0) 
      bufLen = 0;
      
    readChar2Buffer();
  }

  getValuesFromRequest(pSetA, pSetB, pSetC);
  bufLen -= sizeof(tRequest);
  if(bufLen < 0) 
    bufLen = 0;

  return true;

timeout:
  return false;  
}

