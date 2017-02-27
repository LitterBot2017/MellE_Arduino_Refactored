#include"Arduino.h"
#include"PressureSensor.h"

PressureSensor::PressureSensor(int pin)
{

      pinNum= pin;
}

int PressureSensor::GetPressure()
{
    int PressureReading = analogRead(pinNum);
    return PressureReading;
}

bool PressureSensor::GetPickupState()
{
	int val=GetPressure();
	if(val>915)
		return false;
	return true;
}