#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

// Define the motor shield and motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *FW1 = AFMS.getMotor(1); // Front wheel 1
Adafruit_DCMotor *FW2 = AFMS.getMotor(2); // Front wheel 2
Adafruit_DCMotor *BW1 = AFMS.getMotor(3); // Back wheel 1
Adafruit_DCMotor *BW2 = AFMS.getMotor(4); // Back wheel 2

void driveForward();
void turnLeft();
void turnRight();
void stopCar();
bool isObstacleDetected();
void sampleUltrasonicData();
float getUltrasonicDistance();


// Define the servo and ultrasonic sensor pins
Servo servo1;
#define SERVO1_PIN 9
#define SR04_TRIG_PIN 11
#define SR04_ECHO_PIN 12

// Define variables to store ultrasonic sensor data
float center, left, midLeft, midRight, right;

// Define constants
const int stopDistance = 10; // Stop every 10 cm to sample data
const int maxSpeed = 255;   // Maximum motor speed
const int turnSpeed = 150;  // S.peed for turning
const int servoCenter = 90; // Servo center position

void setup() {
  // Initialize the motor shield and servo
  AFMS.begin();
  servo1.attach(SERVO1_PIN);

  // Initialize Serial for debugging
  Serial.begin(9600);
}

void loop() {
  // Move the car straight
  driveForward();

  // Check for obstacles and decide whether to turn
  if (isObstacleDetected()) {
    // Choose the direction with the most space
    if (left > right) {
      turnLeft();
    } else {
      turnRight();
    }

    // Stop and sample ultrasonic sensor data
    stopCar();
    sampleUltrasonicData();

    // Resume forward motion
    driveForward();
  }
}

void driveForward() {
  FW1->setSpeed(maxSpeed);
  FW2->setSpeed(maxSpeed);
  BW1->setSpeed(maxSpeed);
  BW2->setSpeed(maxSpeed);

  FW1->run(FORWARD);
  FW2->run(FORWARD);
  BW1->run(FORWARD);
  BW2->run(FORWARD);
}

void turnLeft() {
  FW1->setSpeed(turnSpeed);
  FW2->setSpeed(turnSpeed);

  FW1->run(BACKWARD);
  FW2->run(BACKWARD);

  BW1->setSpeed(turnSpeed);
  BW2->setSpeed(turnSpeed);

  BW1->run(FORWARD);
  BW2->run(FORWARD);
}

void turnRight() {
  FW1->setSpeed(turnSpeed);
  FW2->setSpeed(turnSpeed);

  FW1->run(FORWARD);
  FW2->run(FORWARD);

  BW1->setSpeed(turnSpeed);
  BW2->setSpeed(turnSpeed);

  BW1->run(BACKWARD);
  BW2->run(BACKWARD);
}

void stopCar() {
  FW1->setSpeed(0);
  FW2->setSpeed(0);
  BW1->setSpeed(0);
  BW2->setSpeed(0);

  FW1->run(RELEASE);
  FW2->run(RELEASE);
  BW1->run(RELEASE);
  BW2->run(RELEASE);

  delay(1000); // Allow the car to stop completely
}

bool isObstacleDetected() {
  float distance = getUltrasonicDistance();
  return distance < stopDistance;
}

void sampleUltrasonicData() {
  servo1.write(0); // Turn the servo to 0 degrees
  delay(500);      // Give time for servo to move

  center = getUltrasonicDistance();

  servo1.write(45); // Turn the servo to 45 degrees
  delay(500);

  midLeft = getUltrasonicDistance();

  servo1.write(90); // Turn the servo to 90 degrees (center)
  delay(500);

  left = getUltrasonicDistance();

  servo1.write(135); // Turn the servo to 135 degrees
  delay(500);

  midRight = getUltrasonicDistance();

  servo1.write(180); // Turn the servo to 180 degrees
  delay(500);

  right = getUltrasonicDistance();

  // Print the sampled data
  Serial.print("Center: ");
  Serial.println(center);
  Serial.print("Left: ");
  Serial.println(left);
  Serial.print("MidLeft: ");
  Serial.println(midLeft);
  Serial.print("MidRight: ");
  Serial.println(midRight);
  Serial.print("Right: ");
  Serial.println(right);
}

float getUltrasonicDistance() {
  // Send a pulse to trigger the ultrasonic sensor
  digitalWrite(SR04_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIG_PIN, LOW);

  // Measure the pulse duration to calculate distance
  float duration = pulseIn(SR04_ECHO_PIN, HIGH);
  float distance = (duration / 2) * 0.0343; // Speed of sound is 343 m/s

  return distance;
}