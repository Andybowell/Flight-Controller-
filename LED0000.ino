/*
  LED

  This example creates a BLE peripheral with service that contains a
  characteristic to control an LED.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic BLE central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>
#include <NewPing.h>
#include <PID_v1.h>
#include <Arduino_LSM9DS1.h>
#include <Servo.h>
#include <HCSR04.h>

#define TRIGGER_PIN  9
#define ECHO_PIN     10
#define MAX_DISTANCE 400

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

Servo right_F_prop;
Servo right_B_prop;
Servo left_F_prop;
Servo left_B_prop;

float duration, distance, C;
float DT = 0.02;        //loop period. i use a loop of 30ms. so DT = 0.03.
float PID, pwmLeft, pwmRight, errors, previous_error, pwm_F_Left, pwm_B_Left, pwm_B_Right, pwm_F_Right;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

/////////////////PID CONSTANTS/////////////////
double kp = 0.2; //3.55
double ki = 0.9; //0.003
double kd = 0.0; //2.05
///////////////////////////////////////////////
double throttle = 1300; //initial value of throttle to the motors
float desired_Height = 35.51; //This is the height in which we want the drone to hover at.
float desired_Height1 = 2.58;
const float sensorRate = 119.00;
int iterations = 5;

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic switchCharacteristic1("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = 11; // pin to use for the LED
const int ledPin1 = 4;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin1, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(ledPin1), landing , RISING);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("LED");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);
  ledService.addCharacteristic(switchCharacteristic1);

  // add service
  BLE.addService(ledService);


  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);
  switchCharacteristic1.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");

  //////////////////////////////////////////////////////////

  // start the filter to run at the sample rate:

  left_B_prop.attach(6); //attatch the left back motor to pin 6
  left_F_prop.attach(3);  //attatch the left front motor to pin 3
  right_F_prop.attach(7); //attach the righ front motor to pin 7
  right_B_prop.attach(5); //attach the righ back motor to pin 5

  // times = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
     of PWM to them before connecting the battery. Otherwise
     the ESCs won't start up or enter in the configure mode.
     The min value is 1000us and max is 2000us, REMEMBER!*/

  left_F_prop.writeMicroseconds(1000);
  left_B_prop.writeMicroseconds(1000);
  right_F_prop.writeMicroseconds(1000);
  right_B_prop.writeMicroseconds(1000);

  delay(7000); /*Give some delay, 7s, to have time to connect
                 the propellers and let everything start up*/
}

void loop() {

  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    Serial.println(" ARMED!!!! ");

    // while the central is still connected to peripheral:
    while (central.connected()) {
      //////////////////////////////////Taking off////////////////////////////////////////////////////////////
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written() || switchCharacteristic1.written()) {
        if (switchCharacteristic.value()) {   // any value other than 0

          Serial.println("LEDred ON");
          digitalWrite(ledPin, HIGH);  // will turn the LED on
          Serial.println("TAKING OFF");

          Takeoff();

        }

        else {                              // a 0 value
          Serial.println(F("LEDred off"));
          digitalWrite(ledPin, LOW); // will turn the LED off

        }

                if (switchCharacteristic1.value()) {   // any value other than 0
        
                  Serial.println("LEDyellow on");
                  digitalWrite(ledPin1, HIGH);  // will turn the LED on
                  Serial.println(F("LANDING!!!!!"));
        
                 // landing();
                }

        else {                              // a 0 value
          Serial.println(F("LEDyellow off"));
          digitalWrite(ledPin1, LOW); // will turn the LED off


        }





        /////////////////////////////////////Landing///////////////////////////////////////////////////////////

        //     if (switchCharacteristic1.written()) {
        //        if (switchCharacteristic1.value()) {   // any value other than 0
        //
        //          Serial.println("LEDyellow on");
        //          digitalWrite(ledPin1, HIGH);  // will turn the LED on
        //          Serial.println(F("LANDING!!!!!"));
        //
        //          landing();
        //        }

        //        else {                              // a 0 value
        //          Serial.println(F("LEDyellow off"));
        //          digitalWrite(ledPin1, LOW); // will turn the LED off
        //
        //        }
      }



    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
