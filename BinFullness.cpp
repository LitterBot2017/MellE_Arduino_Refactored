#include "Arduino.h"
#include "BinFullness.h"

#define MAX_DISTANCE 100

BinFullness::BinFullness(uint8_t triggerPin, uint8_t echoPin)
{
  this->mTriggerPinNum = triggerPin;
  this->mEchoPinNum = echoPin;
  this->mSonar = new NewPing(this->mTriggerPinNum, this->mEchoPinNum, MAX_DISTANCE);
}

int BinFullness::getBinFullness() {
  
  int pingCm = this->mSonar->ping_cm();
  float numerator = BIN_EMPTY_DISTANCE - pingCm;
  float denominator = BIN_EMPTY_DISTANCE - BIN_FULL_DISTANCE;
  
  float distanceFromFull = (numerator/denominator) * 100;
  
  if (distanceFromFull <= 0) {
    return 0;
  } else if (distanceFromFull >= 100) {
    return 100;
  } else {
    return ((int) distanceFromFull);
  }
}
