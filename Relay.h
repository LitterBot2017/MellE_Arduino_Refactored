#ifndef RELAY_H
#define RELAY_H

#include "Arduino.h"

class Relay
{
public:
       Relay(int pin);
       void RelayOn();
       void RelayOff();
private:
       int pinNum;
};
#endif
