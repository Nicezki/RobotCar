#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MS_PWMServoDriver.h>
#include <Servo.h>
#include <SPI.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *FW1 = AFMS.getMotor(1); // Front wheel 1
Adafruit_DCMotor *FW2 = AFMS.getMotor(2); // Front wheel 2
Adafruit_DCMotor *BW1 = AFMS.getMotor(3); // Back wheel 1
Adafruit_DCMotor *BW2 = AFMS.getMotor(4); // Back wheel 2

// We have 2 servo at Pin 9 and 10
#define SERVO1_PIN 9
#define SERVO2_PIN 10

// LCD is hereeeee~
LiquidCrystal_I2C lcd(0x27, 16, 2);



// Define Buzzers
#define BUZZER_PIN 4

Servo servo1;
Servo servo2;

// Obstacle threshold (cm)
#define OBSTACLE_THRESHOLD 10                             
// Ultrasonic sensor
#define SR04_TRIG_PIN 11
#define SR04_ECHO_PIN 12


//QOL Functions
String intToString(int number);



void serialBTControl();
void avoidWall();
void Navigate(String direction);
long getDistance();
void TestNavigate();
void setupLCD();
void setLCDStatus(int mode = 0);
void setLCD(int line, String text);
void servoSet(int degree, int number);
void buzzerControl(int frequency, int delayTime);
void playNote(String note, int delayTime);
void playSound(int melody[], int noteDurations[], int size);
void trackingSpeedToXY(int speed, int degree);
void ServoControl(unsigned long currentmillis, int degree, int number, int Interpolation, int delayTime);
void setServoMicroseconds(int angle, int min, int max, int number);

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;



bool conf_manualMode = false;
bool conf_avoidMode = true;
long duration, distance;
int POS_X = 0, POS_Y = 0;
int center, left, right, midLeft, midRight;

void setupLCD(){
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("RMUTT CPE");
  lcd.setCursor(0, 1);
  lcd.print("ROBOT");
  Serial.println("[OK] Setup LCD");
}

void setLCDStatus(int mode = 0) {
  switch (mode){
    case 0:
      lcd.clear();
      setLCD(0, "D: " + intToString(distance) + " cm");
      setLCD(1, "X:" + intToString(POS_X) + " Y:" + intToString(POS_Y));
      break;
    case 1:
      lcd.clear();
      setLCD(0, " L" + intToString(left) + " C" + intToString(center) + " R" + intToString(right));
      setLCD(1, "  ML" + intToString(midLeft) + "  MR" + intToString(midRight));
      break;
    default:
      break;
  }
}

//Convert int to String with 3 digits
String intToString(int number){
  String result = "";
  if(number < 10){
    result = "00" + String(number);
  }
  else if(number < 100){
    result = "0" + String(number);
  }
  else{
    result = String(number);
  }
  return result;
}

void setLCD(int line, String text) {
  lcd.setCursor(0, line);
  lcd.print(text);
}





void setup() {
  AFMS.begin();
  FW1->setSpeed(20);
  FW2->setSpeed(20);
  BW1->setSpeed(20);
  BW2->setSpeed(20);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  setupLCD();
  
  Serial.begin(115200);
  setLCDStatus();
  Serial.println("[OK] Setup completed");
}

void loop() {
  currentMillis = millis();
  //Check forward distance
  distance = getDistance();
  //Update LCD
  setLCDStatus();

  //SerialBT Control
  serialBTControl();

  //Avoid wall
  if (conf_avoidMode == true) {
    avoidWall();
  }



  //   // TestNavigate();
  // if (currentMillis - previousMillis >= 100) {
  //   previousMillis = currentMillis;
  //     if (conf_manualMode == false) {
  //     Navigate("forward");
  //   }

  //   if (conf_avoidMode == true) {
  //     avoidWall();
  //   }

  // }

}

void serialBTControl() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');  // Read input until newline character
    data.trim();  // Remove leading/trailing whitespaces

    Serial.print("[OK] Received: ");


    if (data.equals("mode switch")) {
      if (conf_manualMode == false) {
        conf_manualMode = true;
        Serial.println("[OK] Manual mode");
      } else {
        conf_manualMode = false;
        Serial.println("[OK] Turn off manual mode");
      }

    } else if (data.equals("avoid switch")) {
        if (conf_avoidMode == false) {
          conf_avoidMode = true;
          Serial.println("[OK] Avoid mode");
        } else {
          conf_avoidMode = false;
          Serial.println("[OK] Turn off avoid mode");
        }

    } else if (data.equals("avoid on")) {
        if (conf_avoidMode == false) {
          conf_avoidMode = true;
          Serial.println("[OK] Avoid mode");
        } else {
          Serial.println("[OK] Avoid mode already on");
        }

    } else if (data.equals("avoid off")) {
      if (conf_avoidMode == true) {
        conf_avoidMode = false;
        Serial.println("[OK] Avoid mode off");
      } else {
        Serial.println("[OK] Avoid mode already off");
      }

    } else if (data.equals("mode man")) {
        if (conf_manualMode == false) {
          conf_manualMode = true;
          Serial.println("[OK] Manual mode");
        } else {
          Serial.println("[OK] Manual mode already on");
        }

    } else if (data.equals("avoid on")){
        if (conf_avoidMode == false) {
          conf_avoidMode = true;
          Serial.println("[OK] Avoid mode");
        } else {
          Serial.println("[OK] Avoid mode already on");
        }

    } else if (data.equals("avoid off")){
        if (conf_avoidMode == true) {
          conf_avoidMode = false;
          Serial.println("[OK] Avoid mode off");
        } else {
          Serial.println("[OK] Avoid mode already off");
        }
    }
    else {
      Serial.println("[ERROR] Invalid command");
    }
  }
  
}

void Navigate(String direction){
  if(direction == "left"){
    FW1->run(FORWARD);
    FW2->run(BACKWARD);
  }
  else if(direction == "right"){
    FW1->run(BACKWARD);
    FW2->run(FORWARD);
  }

  else if(direction == "forward"){
    BW1->run(FORWARD);
    BW2->run(FORWARD);
    FW1->run(FORWARD);
    FW2->run(FORWARD);
  }

  else if(direction == "backward"){
    BW1->run(BACKWARD);
    BW2->run(BACKWARD);
    FW1->run(BACKWARD);
    FW2->run(BACKWARD);
  }
  else if(direction == "stop"){
    FW1->run(RELEASE);
    FW2->run(RELEASE);
    BW1->run(RELEASE);
    BW2->run(RELEASE);
  }
  else{
    Serial.println("[ERROR] Invalid command");
  }

}

void trackingSpeedToXY(int speed, int degree){
  int x = speed * cos(degree);
  int y = speed * sin(degree);
  POS_X += x;
  POS_Y += y;
  Serial.println("POS_X: " + String(POS_X) + " POS_Y: " + String(POS_Y));
}


void avoidWall() {
  static enum { INITIAL_CHECK, GOBACKWARD, TURN_LEFT, TURN_RIGHT, CHECK_OBSTACLE, FINISH } state = INITIAL_CHECK;
  static unsigned long startMillis;

  switch (state) {
    case INITIAL_CHECK:
      distance = getDistance();
      if (distance < OBSTACLE_THRESHOLD) {
        Navigate("stop");
        startMillis = millis();
        state = GOBACKWARD;
      }
      break;

    case GOBACKWARD:
      Navigate("backward");
      if (millis() - startMillis >= 3000 || getDistance() >= 25) {
        Navigate("stop");
        startMillis = millis();
        state = CHECK_OBSTACLE;
      }
      break;


    case TURN_LEFT:
      Navigate("left");
      if (millis() - startMillis >= 3000 || getDistance() >= 40) {
        Navigate("stop");
        state = FINISH;
      }
      break;

    case TURN_RIGHT:
      Navigate("right");
      if (millis() - startMillis >= 3000 || getDistance() >= 40) {
        Navigate("stop");
        state = FINISH;
      }
      break;

    case CHECK_OBSTACLE:
      setLCD(0, "Check obstacle..");
      Navigate("stop");
      // Sample data from 0, 45, 90, 135, 180 degrees with Servo 1
      //Format: center, left, midLeft, midRight, right
      center = getDistance();
      setLCDStatus(1);
      servoSet(0, 1);
      left = getDistance();
      servoSet(135, 1);
      setLCDStatus(1);
      midRight = getDistance();
      servoSet(180, 1);
      setLCDStatus(1);
      right = getDistance();
      servoSet(90, 1);
      midLeft = getDistance();
      setLCDStatus(1);

      if (center >= OBSTACLE_THRESHOLD) {
        // No obstacle in front
        state = INITIAL_CHECK;
      } else {
        // Determine the direction with the most clearance
        if ((left >= right) && (left >= midLeft) && (left >= midRight) && (left >= center) & (left >= OBSTACLE_THRESHOLD)) {
          state = TURN_LEFT;

        } else if ((right >= left) && (right >= midLeft) && (right >= midRight) && (right >= center) & (right >= OBSTACLE_THRESHOLD)) {
          state = TURN_RIGHT;

        } else if ((midLeft >= midRight) && (midLeft >= left) && (midLeft >= right) && (midLeft >= center) & (midLeft >= OBSTACLE_THRESHOLD)) {
          state = TURN_LEFT;

        } else if ((midRight >= midLeft) && (midRight >= left) && (midRight >= right) && (midRight >= center) & (midRight >= OBSTACLE_THRESHOLD)) {
          state = TURN_RIGHT;
        } else {
          state = GOBACKWARD;
        }
      }
      break;

    case FINISH:
      // You might want to add some cleanup or final actions here if needed
      state = INITIAL_CHECK;
      break;
  }
}

long getDistance() {
  digitalWrite(SR04_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SR04_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SR04_TRIG_PIN, LOW);
  duration = pulseIn(SR04_ECHO_PIN, HIGH);
  long old_distance = distance;
  distance = (duration / 2) / 29.1;
  // If distance is more than 500 or less than 2 cm, then it's out of range
  if (distance >= 500 || distance <= 2) {
    Serial.println("[ERROR] Out of range");
    return old_distance;
  }
  // Update the LCD
  setLCDStatus();
  return distance;
  
}

//Degree from 0 - 180
//Number from 1 - 2 (Servo 1 or 2)
void servoControl(unsigned long currentmillis, int degree, int number, int Interpolation = 1, unsigned long delayTime = 15){
  // Smoothly move servo to position
  if (currentmillis - previousMillis >= delayTime) {
    previousMillis = currentmillis;
    for (int pos = 0; pos <= degree; pos += Interpolation) {
      if(number == 1){
        setServoMicroseconds(pos, 1000, 2000, 1);
      }
      else if(number == 2){
        setServoMicroseconds(pos, 900, 1900, 2);
      }
    }
  }
}

//Angle from 0 - 180
//Min and Max from 500 - 2500 (Microseconds)
//Number from 1 - 2 (Servo 1 or 2)
void setServoMicroseconds(int angle, int min, int max, int number){
  if(number == 1){
    servo1.writeMicroseconds(map(angle, 0, 180, min, max));
  }
  else if(number == 2){
    servo2.writeMicroseconds(map(angle, 0, 180, min, max));
  }
}


void servoSet(int degree, int number){
  if(number == 1){
    servo1.write(degree);
  }
  else if(number == 2){
    servo2.write(degree);
  }
}


void playSound(unsigned long currentmillis, int melody[], int noteDurations[], int size) {
  unsigned long previousMillis = currentmillis;

  for (int thisNote = 0; thisNote < size; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    while (currentmillis - previousMillis < noteDuration) {
      currentmillis = millis();
    }
    
    noTone(BUZZER_PIN);
    previousMillis = currentmillis;

    int pauseBetweenNotes = noteDuration * 1.30;
    while (currentmillis - previousMillis < pauseBetweenNotes) {
      currentmillis = millis();
    }
    previousMillis = currentmillis;
  }
}







