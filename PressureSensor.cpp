#include"Arduino.h"
#include"PressureSensor.h"

int history [5] = {0,0,0,0,0};
int counter = 0;

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
    history[counter]=0;
  else
    history[counter]=1;
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += history[i];
  }
  counter = (counter+1)%5;
  return (sum > 4);
}
