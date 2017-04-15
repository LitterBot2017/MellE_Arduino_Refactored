#include <ros.h>
#include <navigation/Arduino.h>
#include <navigation/Navigation.h>
#include <TinyGPS++.h>
#include <RoboClaw.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "PressureSensor.h"
#include "BinFullness.h"
#include "Relay.h"
#include "VoltageSensor.h"

//Watchdog
long last_message = millis();

//Motor stuff
#define mc_address 0x81
RoboClaw motor_control(&Serial2, 10000);

//GPS Stuff
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

float left_motor;
float right_motor;

//Compass Stuff
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//Bin fullness
BinFullness* bin = new BinFullness(48,49);

//PressureSensor
PressureSensor pressure(A5);

//Relay
Relay relay(31);

//Ros node and messages
ros::NodeHandle ros_node;
navigation::Arduino arduino_msg;
navigation::Navigation pc_msg;

// Arduino Publisher
ros::Publisher arduino_pub("arduino", &arduino_msg);

int voltageSensorPin = A0;
VoltageSensor voltageSensor = VoltageSensor(voltageSensorPin);

// PC subscriber callback
void pc_callback (const navigation::Navigation& pc_msg) {

  left_motor = pc_msg.l_motor_val;
  right_motor = pc_msg.r_motor_val;
  last_message = millis();

  motor_control.ForwardBackwardM1(mc_address, left_motor);
  motor_control.ForwardBackwardM2(mc_address, right_motor);

  if(pc_msg.relay_state)
    relay.RelayOn();
  else
    relay.RelayOff();
}

// PC Subscriber
ros::Subscriber <navigation::Navigation> pc_sub("navigation", &pc_callback);

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(GPSBaud);
  if(!mag.begin())
  {
    while(1);
  }
  ros_node.initNode();
  
  ros_node.advertise(arduino_pub);
  
  ros_node.subscribe(pc_sub);
  
  motor_control.begin(38400);
}

void loop() {
  
  //GPS
  while (Serial1.available() > 0)
  
  if (gps.encode(Serial1.read()))
      sendInfo();
  
  // Compass
  sensors_event_t event; 
  mag.getEvent(&event);
  calcHeading(event);

  // Diagnostic Data
  arduino_msg.battery = voltageSensor.getBatteryPercentage();
  arduino_msg.bin_fullness = bin->getBinFullness();
  arduino_msg.pickup_state=pressure.GetPickupState();
  arduino_msg.l_motor = left_motor;
  arduino_msg.r_motor = right_motor;

  // Watchdog
  if(millis()-last_message>1000)
  {
    motor_control.ForwardBackwardM1(mc_address,64);
    motor_control.ForwardBackwardM2(mc_address,64);
  }
  
  // Publish Arduino message on ROS
  arduino_pub.publish(&arduino_msg);
  ros_node.spinOnce();
  delay(100);
}

void calcHeading(sensors_event_t event)
{
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.163;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  arduino_msg.heading=headingDegrees;
}

void sendInfo()
{
  arduino_msg.curr_lat = gps.location.lat();
  arduino_msg.curr_long = gps.location.lng();
  arduino_msg.speed_val = gps.speed.mps();
  arduino_msg.elapsed_time = millis();
  arduino_msg.sats = gps.location.isValid();
}
