#include <AFMotor.h>   // Motor Shield Library
#include <NewPing.h>   // Ultrasonic Sensor Library
#include <Servo.h>     // Servo Library

#define TRIG_PIN A0 
#define ECHO_PIN A1 
#define MAX_DISTANCE 200
#define MAX_SPEED 190   // Maximum motor speed
#define MIN_DISTANCE 15 // Minimum safe distance to obstacle

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);  // Ultrasonic sensor setup

AF_DCMotor motor1(1, MOTOR12_1KHZ);  // Motor 1 on motor shield
AF_DCMotor motor2(2, MOTOR12_1KHZ);  // Motor 2
AF_DCMotor motor3(3, MOTOR34_1KHZ);  // Motor 3
AF_DCMotor motor4(4, MOTOR34_1KHZ);  // Motor 4
Servo myservo;  // Servo motor

int distance = 100;
int speedSet = 0;

void setup() {
  Serial.begin(9600);  // Begin serial communication for debugging
  myservo.attach(10);  // Attach servo on pin 10
  myservo.write(115);  // Set initial servo position
  delay(2000);         // Allow time for initialization

  // Initial distance check
  distance = readPing();
  Serial.print("Initial Distance: ");
  Serial.println(distance);
}

void loop() {
  delay(50);  // Short delay for smooth operation

  // Get the current distance reading
  distance = readPing();
  Serial.print("Current Distance: ");
  Serial.println(distance);

  if (distance > MIN_DISTANCE) {
    // If no obstacle is detected, move forward continuously
    moveForward();
  } else {
    // Stop and handle obstacle
    moveStop();
    delay(100);
    moveBackward();
    delay(300);
    moveStop();
    delay(200);
    handleObstacle();
  }
}

// Servo looks right and checks distance
int lookRight() {
  myservo.write(50); 
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115);  // Reset to center
  return distance;
}

// Servo looks left and checks distance
int lookLeft() {
  myservo.write(170); 
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115);  // Reset to center
  return distance;
}

// Read ultrasonic sensor and return distance
int readPing() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {  // If no object detected, return max distance
    cm = MAX_DISTANCE;
  }
  return cm;
}

// Stop the motors
void moveStop() {
  motor1.run(RELEASE); 
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Move the car forward
void moveForward() {
  motor1.run(FORWARD);      
  motor2.run(FORWARD);
  motor3.run(FORWARD); 
  motor4.run(FORWARD);     
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 10) {  // Increase speed smoothly
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}

// Move the car backward
void moveBackward() {
  motor1.run(BACKWARD);      
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);  
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 10) {  // Increase speed smoothly
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}

// Handle obstacle detection and turning
void handleObstacle() {
  int distanceR = lookRight();   // Look to the right
  delay(200);
  int distanceL = lookLeft();    // Look to the left
  delay(200);

  Serial.print("Distance Right: ");
  Serial.println(distanceR);
  Serial.print("Distance Left: ");
  Serial.println(distanceL);

  // Decide which direction to turn
  if (distanceR >= distanceL) {
    turnRight();
  } else {
    turnLeft();
  }

  // After turning, ensure continuous forward movement
  moveForward();
}

// Turn the car to the right
void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);     
  delay(500);
}

// Turn the car to the left
void turnLeft() {
  motor1.run(BACKWARD);     
  motor2.run(BACKWARD);  
  motor3.run(FORWARD);
  motor4.run(FORWARD);   
  delay(500);
}
