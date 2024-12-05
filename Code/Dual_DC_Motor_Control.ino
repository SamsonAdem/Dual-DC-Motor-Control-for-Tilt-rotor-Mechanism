#include <SparkFun_TB6612.h>
#include <PinChangeInterrupt.h>

#define AIN1 5        // Direction control pin 1 for Motor A
#define AIN2 4        // Direction control pin 2 for Motor A
#define PWMA 9        // PWM to Motor A
#define STBY 8        // Standby pin
#define ENC1 2        // Encoder pin 1
#define ENC2 3        // Encoder pin 2
#define PWM_INPUT_PIN 10  // New PWM input pin

#define LPF_ALPHA 0.05   // LPF smoothing factor (adjust as needed)

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
const int offsetA = 1;

volatile unsigned long lastRise = 0;
volatile unsigned long pulseWidth = 0;
volatile unsigned long filteredPulseWidth = 0;  // Variable to store filtered pulse width
volatile int target = 0;  // target position, scaled -350 to 350
volatile boolean validPWM = false;  // Flag to indicate valid PWM signal

Motor motor1(AIN1, AIN2, PWMA, offsetA, STBY);

void setup() {
  Serial.begin(9600);
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  pinMode(STBY, OUTPUT);  // Ensure STBY pin is configured as output
  digitalWrite(STBY, HIGH); // Set STBY pin high to enable motor driver
  delay(1000);  // Wait for motor driver to stabilize
  
  // Initialize encoder pins and interrupts
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1), readEncoder, RISING);
  
  pinMode(PWM_INPUT_PIN, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PWM_INPUT_PIN), readPWM, CHANGE);
  Serial.println("Filtered PWM Pulse Width");

  // Wait for the serial monitor to open
  while (!Serial);

  // Set filteredPulseWidth to the first value received
  while (!validPWM) {
    delay(10);
  }
  filteredPulseWidth = pulseWidth;
}

void loop() {
  // Apply LPF to the pulseWidth if a valid PWM signal is received
  if (validPWM) {
    filteredPulseWidth = LPF(filteredPulseWidth, pulseWidth, LPF_ALPHA);
  }

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

  // PID calculations if a valid PWM signal is received
  if (validPWM) {
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
  }

  // Debugging output
  Serial.println(filteredPulseWidth);
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
    // Map pulseWidth to a target position
    if (pulseWidth >= 1540 && pulseWidth <= 1560) {
      target = 0; // Set target to zero within the specified range
    } else {
      target = map(filteredPulseWidth, 1000, 2000, -1750/2, 1750/2);
    }
    validPWM = true;  // Set flag to indicate valid PWM signal
  }
}


// Exponential Moving Average (EMA) function for LPF
unsigned long LPF(unsigned long currentValue, unsigned long newValue, float alpha) {
  return (unsigned long)((alpha * newValue) + ((1 - alpha) * currentValue));
}
