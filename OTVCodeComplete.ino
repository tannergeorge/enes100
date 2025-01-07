#include <Enes100.h>
#include <math.h>
#include <Servo.h>

// Motor enable pins
const int leftMotorEnable = 3;
const int rightMotorEnable = 8;

// wheel motor control pins
const int leftMotorForward = 5; // IN1
const int leftMotorBackward = 4; // IN2
const int rightMotorForward = 6; // IN3
const int rightMotorBackward = 7; // IN4

//arm motor control pins
const int armEnable = 2;
const int armDown = 47;
const int armUp = 49;

//servo control
Servo myServo;  // Create a servo object
int servoPin = 9; // Pin connected to the servo's signal

//wifi module pins 
const int tx = 13;
const int rx = 12;

//define distance sensor pins
const int trigPin = 29;    // Trigger
const int echoPin = 27;    // Echo
long duration, inches;

//duty cycle pin
const int dutyPin = 10; //CHANGE

//limit switch pin
const int limitSwitchPin = 39; //CHANGE

// Wi-Fi module setup
void setupWifiModule() {
  Enes100.begin("Data Munchers", DATA, 4, rx,  tx);
  Enes100.println("Connected...");
}

//Distance sensor setup
void distanceSensorSetup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

long getDistance() {
  // Send a 10us pulse to the trigger pin
  digitalWrite(trigPin, LOW);  // Set the trigger pin low
  delayMicroseconds(2);        // Wait for 2 microseconds
  digitalWrite(trigPin, HIGH); // Set the trigger pin high
  delayMicroseconds(10);       // Send a 10-microsecond pulse
  digitalWrite(trigPin, LOW);  // Set the trigger pin low again

  // Read the echo pin and calculate the distance
  duration = pulseIn(echoPin, HIGH);  // Read the echo time in microseconds

  // Convert the time into distance (sound travels at 343 meters per second)
  inches = (duration / 2) / 74;       // Convert to inches

  return inches;
}

// Moving
void enableWheelMotors(int leftSpeed, int rightSpeed) {
  analogWrite(leftMotorEnable, leftSpeed);
  analogWrite(rightMotorEnable, rightSpeed);
}

void enableArm(int speed) {
  analogWrite(armEnable, speed);;
}

void disableWheelMotors() {
  analogWrite(leftMotorEnable, 0);
  analogWrite(rightMotorEnable, 0);
}

void disableArm() {
  analogWrite(armEnable, 0);
}

// Movement functions
void goForward(int leftSpeed, int rightSpeed) {
  enableWheelMotors(leftSpeed, rightSpeed);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

// Move backward
void goReverse(int leftSpeed, int rightSpeed) {
  enableWheelMotors(leftSpeed, rightSpeed);
  
  // Left Motor Backward
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  
  // Right Motor Backward
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

// Turn left (left motor forward, right motor reverse)
void turnLeft(int leftSpeed, int rightSpeed) {
  enableWheelMotors(leftSpeed, rightSpeed);

  // Left Motor Forward
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  
  // Right Motor Backward
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

// Turn right (right motor forward, left motor reverse)
void turnRight(int leftSpeed, int rightSpeed) {
  enableWheelMotors(leftSpeed, rightSpeed);

  // Left Motor Backward
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  
  // Right Motor Forward
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

// Rotate to specific angle and stop when the target is reached
void turnTo(float theta) {
  float t = Enes100.getTheta();
  while(abs(theta - t) >= 0.1) {
    if(theta < 0) {
      turnLeft(150, 155);
    } else {
      turnRight(150, 155);
    }
    t = Enes100.getTheta();
  }
  disableWheelMotors();
}

void turnToZero(bool leftPos) {
  float t = Enes100.getTheta();
  while(abs(M_PI - t) >= 0.1) {
    if(leftPos) {
      turnLeft(150, 155);
    } else {
      turnLeft(150, 155);
    }
    t = Enes100.getTheta();
  }
  disableWheelMotors();
}

void lowerArm(int speed) {
  enableArm(speed);
  digitalWrite(armDown, HIGH);
  digitalWrite(armUp, LOW);
}

void raiseArm(int speed) {
  enableArm(speed);
  digitalWrite(armUp, HIGH);
  digitalWrite(armDown, LOW);
}

void readDutyCycle() {
  unsigned long highTime = pulseIn(dutyPin, HIGH);
  unsigned long lowTime = pulseIn(dutyPin, LOW);

  unsigned long period = highTime + lowTime;
  float dutyCycle = (highTime / (float)period) * 100.0;

  Enes100.mission(CYCLE, (int) dutyCycle);
}

void readLimitSwitch() {
  int switchState = digitalRead(limitSwitchPin);

  if (switchState == LOW) {
    Enes100.mission(MAGNETISM, NOT_MAGNETIC);
  } else {
    Enes100.mission(MAGNETISM, MAGNETIC);
  }
}

void getMagnetism() {
  //openServo();
  readLimitSwitch();
}

void openServo() {
  myServo.write(180);
}

void closeServo() {
  myServo.write(0);
}

void grabPuck() {
  //openServo();
  lowerArm(100);
  delay(3000);
  disableArm();
  //closeServo();
}

void extractPuck() {
  raiseArm(135);
  delay(3000);
  disableArm();
}

//Navigate to the mission site
void navToMission() {
  //Goes to the mission site
  if (Enes100.getY() < 1) {
    turnTo(M_PI/2);
  } else if (Enes100.getY() > 1) {
    turnTo(-M_PI/2);
  }

  disableWheelMotors();
  delay(1000);

  goReverse(150, 150);
  delay(4400);
  disableWheelMotors();
  delay(1000);
}

void completeMission() {
  grabPuck();
  delay(1000);
  readDutyCycle();
  delay(1000);
  extractPuck();
  delay(1000);
  getMagnetism();
}

void navAroundObstacles() {
  bool pastObstacles = false;
  bool hitLeftWall = false;
  int sign = 0;
  long distanceFromObstacle;

  goForward(150, 150);
  delay(4000);
  disableWheelMotors();
  delay(1000);

  turnToZero(Enes100.getY() > 1.0);
  disableWheelMotors();
  delay(1000);


  while (!pastObstacles) {
    distanceFromObstacle = getDistance();

    //Rus if an obstacle is detected
    while (distanceFromObstacle <= 11) {
      //Get around the obstacle
      if (Enes100.getY() > 1) { //0.2 is error
        hitLeftWall = true;
      }

      if (Enes100.getY() <= 1 && !hitLeftWall) {
        disableWheelMotors();
        delay(1000);
        //enableWheelMotors();
        turnTo(-M_PI/2);
        sign = 1;
      } else {
        disableWheelMotors();
        delay(1000);
        //enableWheelMotors();
        turnTo(M_PI/2);
        sign = -1;
      }

      double goalLocation = Enes100.getY() + (sign * 0.35);
      double currLocation = Enes100.getY();

      disableWheelMotors();
      delay(1000);
      //enableWheelMotors();

      while (currLocation < (goalLocation - 0.1) || currLocation > (goalLocation + 0.1)){
        goForward(145, 145);
        currLocation = Enes100.getY();
        delay(500);
      }

      disableWheelMotors();
      delay(1000);
      if (sign == -1) {
        turnToZero(true);
      } else {
        turnTo(M_PI);
      }
      disableWheelMotors();
      delay(1000);
      //enableWheelMotors();
      distanceFromObstacle = getDistance();
      delay(500);
    }

    if (distanceFromObstacle > 11) {
      goForward(145, 145);
    
      pastObstacles = Enes100.getX() >= 2.4;
      hitLeftWall = false;
    }

    pastObstacles = Enes100.getX() >= 2.4;
    delay(500);
  }
}

void navToFinish() {
  if (Enes100.getY() > 0.4) {
    turnTo(M_PI/2);
    while (Enes100.getY() > 0.4) {
      goForward(150, 150);
    }
  }

  turnTo(0);
  goReverse(150, 150);
  delay(500);
  goReverse(255, 255);
  delay(5000);
}

void setup() {
  distanceSensorSetup();
  setupWifiModule();
  //myServo.attach(servoPin);
  int motorPins[] = {leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward, leftMotorEnable, rightMotorEnable};
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
  Serial.begin(9600);

  navToMission();
  completeMission();
  navAroundObstacles();
  navToFinish();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print(getDistance());
}