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
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif


// SFEVL53L1X distanceSensor;
// Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
// SFEVL53L1X distanceSensor1;
// SFEVL53L1X distanceSensor2;

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "8fe22c06-7653-4f53-b558-bd850e74b4f4"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 1000;
static long previousMillis = 0;
unsigned long currentMillis = 0;
//////////// Global Variables ////////////
/* Computation variables */
float pitch_a = 0, roll_a = 0, pitch_g = 0, roll_g = 0, yaw_g = 0, dt =0, pitch = 0, roll = 0, yaw = 0;
float Xm = 0, Ym =0, Zm = 0, x = 0, y = 0;
unsigned long last_time = millis();
double pitch_a_LPF[] = {0, 0};
double pitch_a_LPF[] = {0, 0};
const int n=1;
volatile bool ranging = 0;
const float alpha = 0.2;

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
    delay(50);
    Serial.println("Sensor1 failed to begin. Please check wiring. Freezing...");
    // while (1)
    //   ;
  }
  Serial.println("TOF1 online");

  while (distanceSensor2.begin(Wire) != 0) //Begin returns 0 on a good init
  {
    delay(50);
    Serial.println("Sensor2 failed to begin. Please check wiring. Freezing...");
    // while (1)
    //   ;
  }
  Serial.println("TOF22 online");
  
  distanceSensor1.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();
  Serial.println("setup over!");
  Serial.println("------------");

  // ---------
  // BLE setup
  // ---------
  BLE.begin();
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);
  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on central devices
  tx_characteristic_float.writeValue(0.0);

  /*
    * An example using the EString
    */
  // Clear the contents of the EString before using it
  tx_estring_value.clear();

  // Append the string literal "[->"
  tx_estring_value.append("[->");

  // Append the float value
  tx_estring_value.append(9.0);

  // Append the string literal "<-]"
  tx_estring_value.append("<-]");

  // Write the value to the characteristic
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();

  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  // Blink LED
  for (int i = 0; i < 3; i ++) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(200);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(200);                       // wait for a second
  }
}

void
handle_command()
{
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                            rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
    * since it uses strtok internally (refer RobotCommand.h and 
    * https://www.cplusplus.com/reference/cstring/strtok/)
    */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }

  switch (cmd_type) {
    case 0: {
      // use a global variable to see if sensor is already on
      if (ranging == 0) {
        distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
        distanceSensor2.startRanging();
        ranging = 1;
      }

      // Serial.println("waiting for data...");
      if ( !(distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady() && myICM.dataReady()) )
      {
        return;
      }

      // fetch TOF
      int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
      int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor1.clearInterrupt();
      distanceSensor1.stopRanging();
      distanceSensor2.clearInterrupt();
      distanceSensor2.stopRanging();
      ranging = 0;

      // fetch ICM
      myICM.getAGMT();       
    
      long int t_ms = millis();    
      char t_str[10];
      itoa(t_ms, t_str, 10);

      pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
      roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
      pitch_g = (pitch_g+myICM.gyrX()*dt)*0.9 + pitch_a*0.1;
      roll_g = (roll_g+myICM.gyrX()*dt)*0.9 + roll_a*0.1;
      

      pitch_a_LPF[n] = alpha*pitch_a + (1-alpha)*pitch_a_LPF[n-1];
      pitch_a_LPF[n-1] = pitch_a_LPF[n];

      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(t_str);
      tx_estring_value.append("|TOF1:");
      tx_estring_value.append(distance1);
      tx_estring_value.append("|TOF2:");
      tx_estring_value.append(distance2);
      tx_estring_value.append("|AP:");
      tx_estring_value.append(pitch_a);
      tx_estring_value.append("|AR:");
      tx_estring_value.append(roll_a);
      tx_estring_value.append("|GP:");
      tx_estring_value.append(pitch_g);
      tx_estring_value.append("|GR:");
      tx_estring_value.append(roll_g);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      // Serial.print("Sent back: ");
      // Serial.println(tx_estring_value.c_str());

      break;
    }

    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
  // TODO
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void loop(void)
{
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
      Serial.print("Connected to: ");
      Serial.println(central.address());

      // While central is connected
      while (central.connected()) {
          // Send data
          write_data();

          // Read data
          read_data();
      }

      Serial.println("Disconnected");
  }

}