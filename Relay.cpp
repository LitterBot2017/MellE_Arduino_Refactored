#include"Arduino.h"
#include"Relay.h"

Relay::Relay(int pin)
{
      pinNum= pin;
      pinMode(pin,OUTPUT);
}

void Relay::RelayOn()
{
	digitalWrite(pinNum,HIGH);
}

void Relay::RelayOff()
{
	digitalWrite(pinNum,LOW);
}
