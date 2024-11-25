#include <SparkFun_TB6612.h>

#define AIN1 A2  // Direction control pin 1 for Motor A (analog pin 2)
#define AIN2 A3  // Direction control pin 2 for Motor A (analog pin 3)
#define PWMA 9
#define STBY 8

const int offsetA = 1;

Motor motor1(AIN1, AIN2, PWMA, offsetA, STBY); // Adjusted constructor for library usage

void setup() {
  pinMode(STBY, OUTPUT);
  // Activate the motor driver by setting STBY to HIGH
  digitalWrite(STBY, HIGH);
}

void loop() {
  digitalWrite(STBY, HIGH);

  motor1.brake();  // Brake for instant stop
  delay(2000);     // Brake for 2 seconds
  
  // Drive Motor A forward at full speed
  motor1.drive(255);  // Full speed forward
  delay(20000/12);       // Run for 2 seconds

  motor1.brake();  // Brake for instant stop
  delay(2000);     // Brake for 2 seconds

  motor1.drive(-255);  // Full speed backward
  delay(20000/12);       // Run for 2 seconds
  
  // Brake Motor A
  motor1.brake();  // Brake for instant stop
  delay(2000);     // Brake for 2 seconds
}
