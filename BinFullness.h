#ifndef BINFULLNESS_H
#define BINFULLNESS_H

#include "Arduino.h"
#include <NewPing.h>

#define MAX_DISTANCE 100
#define BIN_EMPTY_DISTANCE 66
#define BIN_FULL_DISTANCE 38

class BinFullness
{
  public:
    // Note triggerPin = green, echoPin = orange
    BinFullness(uint8_t triggerPin, uint8_t echoPin);
    int getBinFullness();
    
  private:
    uint8_t mTriggerPinNum;
    uint8_t mEchoPinNum;
    NewPing* mSonar;
    
};
#endif

