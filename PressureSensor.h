#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#include "Arduino.h"

class PressureSensor
{
public:

       PressureSensor(int pin);
       int GetPressure();
       bool GetPickupState();
private:
       int pinNum;
};
#endif