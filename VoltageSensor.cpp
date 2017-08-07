#include "Arduino.h"
#include "VoltageSensor.h"

#define address 0x81

RoboClaw roboclaw(&Serial2,10000);

VoltageSensor::VoltageSensor(int pin) {
  this->pinNum = pin;
};


int VoltageSensor::getRoboClawBatteryPercentage() {
  int sensorValue = roboclaw.ReadMainBatteryVoltage(address)/10.0;
  int index = 0;
  while (index < sizeof(this->voltages) && this->voltages[index] < sensorValue) {
    index++;
  }
  if (index <= 0){
    return percentFull[0];
  } else {
    return percentFull[index - 1];
  }
}

float VoltageSensor::getBatteryVoltage() {
  return this->voltages[this->getBatteryIndex()];
}

int VoltageSensor::getBatteryPercentage() {
  return this->percentFull[this->getBatteryIndex()];
}

int VoltageSensor::getBatteryIndex() {
  int sensorValue = this->getSensorValue();
  int index = 0;
  while (index < sizeof(this->sensorValues) && this->sensorValues[index] < sensorValue) {
    index++;
  }
  if (index <= 0){
    return 0;
  } else {
    return (index - 1);
  }
}

int VoltageSensor::getSensorValue() {
  return analogRead(this->pinNum);
}

