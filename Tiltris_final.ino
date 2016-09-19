
/* Program: Tiltis
 *  Authors: Carrina Dong, Caitlin Gruis, Lara Janse van Vuuren
 *  Date: 9/19/2016
 *  Purpose: This program takes sensor information from an accelerometer, FSR, photocell, and flex sensor and displays the result to the serial monitor
 */

#if defined(ARDUINO) 
SYSTEM_MODE(MANUAL); 
#endif

#include "application.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

//Initialize Accelerometer
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

//Initialize FSR variables
int FSRpin = A0;
int val;
int thresh1=50;

//Initialize Photocell Variables
int photocellPin = A1;
int photocellReading;

//Initialize Flex Sensor Variables
int flexSensorPin = A2; 
int flexSensorReading; 

void setup(void) {

  Serial.begin(9600);
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative accelerometer i2c address
    while (1);
  }
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  pinMode(FSRpin, INPUT); // initialize FSR sensor as input
  pinMode(photocellPin, INPUT); //initialize photoresistor as input
  
}

void loop() {

  //Accelerometer Readings
  lis.read();      // get X Y and Z data at once
  sensors_event_t event; 
  lis.getEvent(&event);
  //Serial.println(event.acceleration.z);
  if (event.acceleration.z <= 6.00) {
    Serial.print("DOWN.");
    Serial.print("DOWN.");
  }

  //FSR Readings
  val = analogRead(FSRpin);
  if (val > thresh1){
    Serial.print("UP.");
  }

  //Photocell Readings
  photocellReading = analogRead(photocellPin);
  //Serial.println(photocellReading);
  if(photocellReading > 2500){
    Serial.print("LEFT.");
    delay(100);
  }
  if(photocellReading < 25){
    Serial.print("START.");
    delay(100);
  }

  //Flex Sensor Readings
   flexSensorReading = analogRead(flexSensorPin);
  if(flexSensorlReading > 2500){
    Serial.print("LEFT.");
    delay(100);
  }
  if(flexSensorlReading < 2500){
    Serial.print("RIGHT.");
    delay(100);
  }
  delay(300); 
}

