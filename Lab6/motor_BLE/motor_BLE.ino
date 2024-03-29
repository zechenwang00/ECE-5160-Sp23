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

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 5
#define INTERRUPT_PIN 4

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

// motor vars
#define RIGHT_FWD  2
#define RIGHT_RWD  3
#define LEFT_FWD   14
#define LEFT_RWD   15 
#define OFFSET     25
#define BK_OFFSET  60
#define BASE_SPEED 50

// 2 time lists using int and 2 data lists using short, using 12 * LIST_SIZE / 1024 RAM. Max LIST_SIZE ~= 30k.
// can be improved if saving relative time using smaller data types, or only use 1 time list assuming synchronized
#define LIST_SIZE  2048

// Number of data each transmission includes
#define TRANSMIT_SIZE 8

//////////// Global Variables ////////////
short tof1_list[3] = {0,0,0};
short tof_data_list[LIST_SIZE], pwm_data_list[LIST_SIZE];
int   tof_time_list[LIST_SIZE], pwm_time_list[LIST_SIZE];
short data_idx = 0;

bool init_pid = 0;
int setpoint = 300;
float pid_i, pid_d;
long int pid_last_time = 0;
float kp = 0.06;
float ki = 0.01;
float kd = 0.03;
float pid_alpha = 0.5;
bool pid_running = 0;
bool car_running = 0;

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
  Serial.println("TOF1 online");

  while (distanceSensor2.begin(Wire) != 0) //Begin returns 0 on a good init
  {
    delay(500);
    Serial.println("Sensor2 failed to begin. Please check wiring. Freezing...");
    // while (1)
    //   ;
  }
  Serial.println("TOF2 online");
  
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

  // motor
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_RWD, OUTPUT);
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_RWD, OUTPUT);

  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 0);
}

void motor_stop() {
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_RWD, 0);
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_RWD, 0);
}

void motor_both(int duty) {
  short motor_signal;
  if (duty < 0) {
    // negative means forward
    motor_signal = min(100, OFFSET + abs(duty));
    analogWrite(RIGHT_FWD, motor_signal);
    analogWrite(LEFT_FWD,  motor_signal);
    analogWrite(RIGHT_RWD, 0);
    analogWrite(LEFT_RWD,  0);
  } else {
    // positive means backwards :)
    motor_signal = min(100, BK_OFFSET + abs(duty));
    analogWrite(RIGHT_RWD, motor_signal);
    analogWrite(LEFT_RWD,  motor_signal);
    analogWrite(RIGHT_FWD, 0);
    analogWrite(LEFT_FWD,  0);
  }
  // data recording
  if (data_idx < LIST_SIZE) {
    pwm_data_list[data_idx] = motor_signal;
    pwm_time_list[data_idx] = millis();
  }

}

void motor_start_fwd() {
    analogWrite(RIGHT_FWD, 70);
    analogWrite(LEFT_FWD,  70);
    analogWrite(RIGHT_RWD, 0);
    analogWrite(LEFT_RWD,  0);  
}

void motor_slow_fwd() {
    analogWrite(RIGHT_FWD, 40);
    analogWrite(LEFT_FWD,  40);
    analogWrite(RIGHT_RWD, 0);
    analogWrite(LEFT_RWD,  0);  
}

void send_data() {
  short curr_idx = 0;

  // send TOF
  while ((curr_idx < LIST_SIZE) && !(curr_idx > data_idx)) {
    tx_estring_value.clear();
    // Specify transmitting ToF data
    tx_estring_value.append("T");
    tx_estring_value.append("|");
    // Transmit timestamped data
    for(char i = 0; i < TRANSMIT_SIZE; i++) {
      tx_estring_value.append(tof_time_list[curr_idx+i]);
      tx_estring_value.append("&");
      tx_estring_value.append(tof_data_list[curr_idx+i]);
      tx_estring_value.append(",");
    }
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    // Serial.print("Sent back: ");
    // Serial.println(tx_estring_value.c_str());
    curr_idx += TRANSMIT_SIZE;
  }

  // send PWM
  curr_idx = 0;
  while ((curr_idx < LIST_SIZE) && !(curr_idx > data_idx)) {
    tx_estring_value.clear();
    tx_estring_value.append("P");
    tx_estring_value.append("|");
    for(char i = 0; i < TRANSMIT_SIZE; i++) {
      tx_estring_value.append(pwm_time_list[curr_idx+i]);
      tx_estring_value.append("&");
      tx_estring_value.append(pwm_data_list[curr_idx+i]);
      tx_estring_value.append(",");
    }
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    // Serial.print("Sent back: ");
    // Serial.println(tx_estring_value.c_str());
    curr_idx += TRANSMIT_SIZE;
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
      // // sensor
      // distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
      // distanceSensor2.startRanging();

      // Serial.println("waiting for data...");
      // while ( !(distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady()) )
      // {
      //   delay(1);
      // }

      // int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
      // int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
      // distanceSensor1.clearInterrupt();
      // distanceSensor1.stopRanging();
      // distanceSensor2.clearInterrupt();
      // distanceSensor2.stopRanging();

      // //get time
      // long int t_ms = millis();    
      // char t_str[10];
      // itoa(t_ms, t_str, 10);

      // // init tof if needed
      // if (init_pid == 0) {
      //   tof1_list[2] = distance1;
      //   tof1_list[1] = distance1;
      //   tof1_list[0] = distance1;
        
      //   pid_last_time = t_ms;
      //   pid_i = 0;
      //   pid_d = 0;
        
      //   memset(tof_data_list, 0, sizeof(tof_data_list));
      //   memset(tof_time_list, 0, sizeof(tof_time_list));
      //   memset(pwm_data_list, 0, sizeof(pwm_data_list));
      //   memset(pwm_time_list, 0, sizeof(pwm_time_list));

      //   init_pid = 1;

      // } else {
      //   tof1_list[0] = tof1_list[1];
      //   tof1_list[1] = tof1_list[2];
      //   tof1_list[2] = distance1;
      // }

      // // calculate err
      // float pid_dt;
      // pid_dt = (int)(t_ms - pid_last_time) / 1000.0;
      // //p
      // int pid_e;
      // pid_e = setpoint - distance1;
      // //i
      // pid_i += pid_e * pid_dt;
      // //d
      // pid_d = pid_alpha * pid_d + (1 - pid_alpha) * (tof1_list[1] - tof1_list[0]) / pid_dt;

      // //pwm signal
      // float pid_pwm;
      // pid_pwm = kp * pid_e + ki * pid_i + kd * pid_d;

      // // update current itr
      // pid_last_time = t_ms;

      // tx_estring_value.clear();
      // tx_estring_value.append(t_str);
      // tx_estring_value.append(",");
      // tx_estring_value.append(distance1);
      // tx_estring_value.append(",");
      // tx_estring_value.append(distance2);
      // tx_estring_value.append(",");
      // tx_estring_value.append(pid_dt);
      // tx_estring_value.append(",");
      // tx_estring_value.append(pid_e);
      // tx_estring_value.append(",");
      // tx_estring_value.append(pid_i);
      // tx_estring_value.append(",");
      // tx_estring_value.append(pid_d);
      // tx_estring_value.append(",");
      // tx_estring_value.append(pid_pwm);
      // tx_estring_value.append("|");
      // tx_characteristic_string.writeValue(tx_estring_value.c_str());

      // Serial.print("Sent back: ");
      // Serial.println(tx_estring_value.c_str());

      break;
    }

    case 1:
      pid_running = 1;
      // motor_start_fwd();
      // delay(500);
      // motor_slow_fwd();
      break;

    case 2:
      pid_running = 0;
      init_pid = 0;
      motor_stop();
      send_data();
      break;
    
    case 3:
      motor_start_fwd();
      break;
    
    case 4:
      motor_slow_fwd();
      break;

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

        if (pid_running) {
          distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
          distanceSensor2.startRanging();

          // Serial.println("waiting for data...");
          while ( !(distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady()) )
          {
            // delay(1);
          }

          int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
          int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
          if (distance1 == 0) {
            distance1 = tof1_list[2];
          }
          distanceSensor1.clearInterrupt();
          distanceSensor1.stopRanging();
          distanceSensor2.clearInterrupt();
          distanceSensor2.stopRanging();
          


          //get time
          int t_ms = millis();    
          // char t_str[10];
          // itoa(t_ms, t_str, 10);

          // init tof if needed
          if (init_pid == 0) {
            tof1_list[2] = distance1;
            tof1_list[1] = distance1;
            tof1_list[0] = distance1;
            pid_last_time = t_ms;
            init_pid = 1;
          } else {
            tof1_list[0] = tof1_list[1];
            tof1_list[1] = tof1_list[2];
            tof1_list[2] = distance1;
          }

          // calculate err
          float pid_dt;
          pid_dt = (int)(t_ms - pid_last_time) / 1000.0;
          //p
          int pid_e;
          pid_e = setpoint - distance1;
          //i
          pid_i += pid_e * pid_dt;
          if(pid_i > 1000) {
            pid_i = 1000;
          } else if(pid_i < -1000) {
            pid_i = -1000;
          }
          //d
          pid_d = (tof1_list[1] - tof1_list[0]) / pid_dt;

          //pwm signal
          float pid_pwm;
          pid_pwm = kp * pid_e + ki * pid_i - kd * pid_d;

          // update current itr
          pid_last_time = t_ms;
          
          motor_both(round(pid_pwm));

          if(data_idx < LIST_SIZE) {
            tof_data_list[data_idx] = distance1;
            tof_time_list[data_idx] = t_ms;
            data_idx += 1;
          } 

        }

      }

      Serial.println("Disconnected");
      motor_stop();
  }

}
