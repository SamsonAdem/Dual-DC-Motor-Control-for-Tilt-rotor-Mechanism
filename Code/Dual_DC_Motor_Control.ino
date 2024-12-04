#include <SparkFun_TB6612.h>
#include <PinChangeInterrupt.h>

#define AIN1 5  // Direction control pin 1 for Motor A
#define AIN2 4  // Direction control pin 2 for Motor A
#define PWMA 9  // PWM to Motor A
#define STBY 8  // Standby
#define ENC1 2  // Encoder pin 1
#define ENC2 3  // Encoder pin 2
#define PWM_INPUT_PIN 10  // New PWM input pin

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
const int offsetA = 1;

volatile unsigned long lastRise = 0;
volatile unsigned long pulseWidth = 0;
volatile int target = 0;  // target position, scaled -350 to 350

Motor motor1(AIN1, AIN2, PWMA, offsetA, STBY);

void setup() {
  Serial.begin(9600);
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  pinMode(PWM_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1), readEncoder, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PWM_INPUT_PIN), readPWM, CHANGE);
  Serial.println("Target Pos");
}

void loop() {
  // Read the position
  int pos = 0;
  noInterrupts();  // Disable interrupts temporarily while reading
  pos = posi;
  interrupts();  // Re-enable interrupts

  // PID constants
  float kp = 10;
  float kd = 0.25;
  float ki = 0.25;

  // Calculate time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;

  // Error calculation
  int e = pos - target;

  // PID calculations
  float dedt = (e - eprev) / deltaT;
  eintegral += e * deltaT;
  float u = kp * e + kd * dedt + ki * eintegral;

  // Motor power and direction
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  int dir = u < 0 ? -1 : 1;

  // Drive motor
  motor1.drive(pwr * dir);

  // Store previous error
  eprev = e;

  // Debugging output
  Serial.print("Target: ");
  Serial.print(target);
  Serial.print(" Current Pos: ");
  Serial.println(pos);
}

void readEncoder() {
  int b = digitalRead(ENC2);
  if (b > 0) {
    posi++;
  } else {
    posi--;
  }
}

void readPWM() {
  if (digitalRead(PWM_INPUT_PIN) == HIGH) {
    lastRise = micros();
  } else {
    pulseWidth = micros() - lastRise;
    // Map pulseWidth to a target position, -350 for 1000 us and 350 for 2000 us
    target = map(pulseWidth, 1000, 2000, -1750/2, 1750/2);
  }
}
