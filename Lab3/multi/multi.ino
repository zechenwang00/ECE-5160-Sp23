/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

// SFEVL53L1X distanceSensor;
// Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
// SFEVL53L1X distanceSensor1;
// SFEVL53L1X distanceSensor2;

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);
  distanceSensor2.setI2CAddress(0x3f);
  digitalWrite(SHUTDOWN_PIN, HIGH);

  while (distanceSensor1.begin(Wire) != 0) //Begin returns 0 on a good init
  {
    delay(500);
    Serial.println("Sensor1 failed to begin. Please check wiring. Freezing...");
    // while (1)
    //   ;
  }
  Serial.println("Sensor1 online!");

  while (distanceSensor2.begin(Wire) != 0) //Begin returns 0 on a good init
  {
    delay(500);
    Serial.println("Sensor2 failed to begin. Please check wiring. Freezing...");
    // while (1)
    //   ;
  }
  Serial.println("Sensor2 online!");
  
  distanceSensor1.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();
  Serial.println("setup over!");

}

void loop(void)
{
  distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
  distanceSensor2.startRanging();
  // Serial.println("ranging");
  while ( !(distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady()) )
  {
    delay(1);
    // Serial.println(millis());
  }
  int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
  int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor1.clearInterrupt();
  distanceSensor1.stopRanging();
  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();

  Serial.print("TOF1:");
  Serial.print(distance1);
  Serial.print("|TOF2:");
  Serial.print(distance2);

  // float distanceInches = distance * 0.0393701;
  // float distanceFeet = distanceInches / 12.0;

  // Serial.print("\tDistance(ft): ");
  // Serial.print(distanceFeet, 2);

  Serial.println();
}
