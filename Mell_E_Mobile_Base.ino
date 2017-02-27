#include <ros.h>
#include <melle_refactored/MellE_msg.h>
#include <melle_refactored/PC_msg.h>
#include <TinyGPS++.h>
#include <RoboClaw.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "PressureSensor.h"
#include "BinFullness.h"

//Watchdog
long last_message = millis();

//Motor stuff
#define mc_address 0x81
RoboClaw motor_control(&Serial2, 10000);

//GPS Stuff
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
float des_lat;
float des_long;
int waypoint_id;
float left_motor;
float right_motor;

//Compass Stuff
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//Bin fullness
BinFullness* bin = new BinFullness(48,49);

//PressureSensor
PressureSensor pressure(A5);

//Ros node and messages
ros::NodeHandle executor_node;
melle_refactored::MellE_msg data_msg;
melle_refactored::PC_msg msg_received;

//Publisher stuff
ros::Publisher executor_pub("MellE_msg", &data_msg);

//Subscriber stuff
void pc_callback (const melle_refactored::PC_msg& msg)
{
  des_lat = msg.dest_lat;
  des_long = msg.dest_long;
  waypoint_id = msg.waypoint_id;
  left_motor = msg.l_motor_val;
  right_motor = msg.r_motor_val;
  last_message = millis();
  motor_control.ForwardBackwardM1(mc_address,left_motor);
  motor_control.ForwardBackwardM2(mc_address,right_motor);
}
ros::Subscriber <melle_refactored::PC_msg> pc_msg_subs("PC_msg", &pc_callback);

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(GPSBaud);
  if(!mag.begin())
  {
    while(1);
  }
  executor_node.initNode();
  executor_node.advertise(executor_pub);
  executor_node.subscribe(pc_msg_subs);
  motor_control.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  //GPS
  while (Serial1.available() > 0)
  if (gps.encode(Serial1.read()))
      sendInfo();
  
  //Compass
  sensors_event_t event; 
  mag.getEvent(&event);
  calcHeading(event);

  //Diagnostic Data
  data_msg.battery = 10;
  data_msg.bin_fullness = bin->getBinFullness();
  data_msg.pickup_state=pressure.GetPickupState();
  data_msg.l_motor = left_motor;
  data_msg.r_motor = right_motor;

  //watchdog
  if(millis()-last_message>1000)
  {
    motor_control.ForwardBackwardM1(mc_address,64);
    motor_control.ForwardBackwardM2(mc_address,64);
  }
  
  //Ros publish
  executor_pub.publish(&data_msg);
  executor_node.spinOnce();
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
  data_msg.heading=headingDegrees;
}

void sendInfo()
{
  data_msg.curr_lat=gps.location.lat();
  data_msg.curr_long=gps.location.lng();
  data_msg.speed_val=gps.speed.mps();
  data_msg.elapsed_time=millis();
  data_msg.sats=gps.location.isValid();
}
