#include <SparkFun_TB6612.h>
#include <PinChangeInterrupt.h>

// Motor A pins
#define AIN1 A1       // Direction control pin 1 for Motor A
#define AIN2 A2       // Direction control pin 2 for Motor A
#define AENC1 2       // Encoder pin 1 for Motor A --- interrupt pin
#define AENC2 4       // Encoder pin 2 for Motor A
#define PWMA 5        // PWM to Motor A

// Motor B pins
#define BIN1 A3       // Direction control pin 1 for Motor B
#define BIN2 A4       // Direction control pin 2 for Motor B
#define BENC1 3       // Encoder pin 1 for Motor B --- interrupt pin
#define BENC2 7       // Encoder pin 2 for Motor B
#define PWMB 6        // PWM to Motor B

// Common pins 
#define STBY 12       // Standby pin
#define PWM_INPUT_PIN 10  // New PWM input pin from RC

#define LPF_ALPHA 0.05   // LPF smoothing factor (adjust as needed)

volatile int posiA = 0;
volatile int posiB = 0;
long prevT = 0;
float eprevA = 0, eprevB = 0;
float eintegralA = 0, eintegralB = 0;
const int offsetA = 1, offsetB = -1;

volatile unsigned long lastRise = 0;
volatile unsigned long pulseWidth = 0;
volatile unsigned long filteredPulseWidth = 0;  // Variable to store filtered pulse width
volatile int target = 0;  // Target position, scaled -350 to 350
volatile boolean validPWM = false;  // Flag to indicate valid PWM signal

Motor motorA(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorB(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
  Serial.begin(9600);
  pinMode(AENC1, INPUT);
  pinMode(AENC2, INPUT);
  pinMode(BENC1, INPUT);
  pinMode(BENC2, INPUT);
  pinMode(STBY, OUTPUT);  
  digitalWrite(STBY, HIGH); 
  delay(1000);  
  
  attachInterrupt(digitalPinToInterrupt(AENC1), readEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(BENC1), readEncoderB, RISING);
  
  pinMode(PWM_INPUT_PIN, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PWM_INPUT_PIN), readPWM, CHANGE);
  Serial.println("Filtered PWM Pulse Width"); 
}

void loop() { 
  if (validPWM) {
    filteredPulseWidth = LPF(filteredPulseWidth, pulseWidth, LPF_ALPHA);
    
    int posA = 0, posB = 0;
    noInterrupts();
    posA = posiA;
    posB = posiB;
    interrupts();

    float kp = 10, kd = 0.25, ki = 0.25;

    float kpB = 10;  // Adjust these values as necessary for tuning
    float kdB = 0.25;
    float kiB = 0.25;


    long currT = micros();
    float deltaT = ((float)(currT - prevT)) / 1.0e6;
    prevT = currT;

    int eA = posA - target, eB = target - posB ;
    calculateAndDriveMotor(eA, eprevA, eintegralA, motorA, deltaT, kp, kd, ki);
    calculateAndDriveMotor(eB, eprevB, eintegralB, motorB, deltaT, kpB, kdB, kiB);

    Serial.println(filteredPulseWidth);
  }
}

void readEncoderA() {
  int bA = digitalRead(AENC2);
  if (bA > 0) {
    posiA++;
  } else {
    posiA--;
  }
}

void readEncoderB() {
  int bB = digitalRead(BENC2);
  if (bB > 0) {
    posiB++;
  } else {
    posiB--;
  }
}

void readPWM() {
  if (digitalRead(PWM_INPUT_PIN) == HIGH) {
    lastRise = micros();
  } else {
    pulseWidth = micros() - lastRise;
    if (pulseWidth >= 1540 && pulseWidth <= 1560) {
      target = 0;
    } else {
      target = map(filteredPulseWidth, 1000, 2000, -1750/2, 1750/2);
    }
    validPWM = true;
    }
  }

// Calculate and drive the motor using PID control
void calculateAndDriveMotor(int error, float &prevError, float &integralError, Motor &motor, float deltaTime, float kp, float kd, float ki) {
  float derivative = (error - prevError) / deltaTime;  // Calculate the derivative of the error
  integralError += error * deltaTime;  // Update the integral of the error

  // Calculate the control variable (PID output)
  float controlSignal = kp * error + kd * derivative + ki * integralError;

  // Determine the power output and direction
  float power = fabs(controlSignal);  // Power is the absolute value of control variable
  if (power > 255) power = 255;  // Limit power to max PWM value of 255
  int direction = controlSignal < 0 ? -1 : 1;  // Direction based on the sign of the control variable

  // Drive the motor with calculated power and direction
  motor.drive(power * direction);

  // Update the previous error
  prevError = error;
}

// Exponential Moving Average (EMA) function for LPF
unsigned long LPF(unsigned long currentValue, unsigned long newValue, float alpha) {
  return (unsigned long)((alpha * newValue) + ((1 - alpha) * currentValue));
}

// updated contorl logic will be done on the next step
