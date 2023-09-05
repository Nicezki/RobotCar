#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MS_PWMServoDriver.h>
#include <Servo.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
 
// [CONFIG]
// <Wifi (Hotspot)>
#define WIFI_SSID "CPE Robot Car"
#define WIFI_PASSWORD "12345678"

// <Servo Pin>
// Front Servo
// Servo 1 is used for Ultrasonic sensor
#define SERVO1_PIN 9
// Back Servo (Unused for now)
// Servo 2 is not used for now
#define SERVO2_PIN 10

// <Speaker Pin>
// Buzzer that will make sound when the car is moving
#define BUZZER_PIN 4

// Obstacle threshold (cm)
// Before this distance, the car will stop
#define OBSTACLE_THRESHOLD 10

// Ultrasonic sensor
#define SR04_TRIG_PIN 11
#define SR04_ECHO_PIN 12



ESP8266WebServer server(80);


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *FW1 = AFMS.getMotor(1); // Front wheel 1
Adafruit_DCMotor *FW2 = AFMS.getMotor(2); // Front wheel 2
Adafruit_DCMotor *BW1 = AFMS.getMotor(3); // Back wheel 1
Adafruit_DCMotor *BW2 = AFMS.getMotor(4); // Back wheel 2



// LCD is hereeeee~
LiquidCrystal_I2C lcd(0x27, 16, 2);





Servo servo1;
Servo servo2;





