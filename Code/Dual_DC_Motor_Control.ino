#include <SparkFun_TB6612.h>

#define AIN1 5  // Direction control pin 1 for Motor A
#define AIN2 4  // Direction control pin 2 for Motor A
#define PWMA 9
#define STBY 8
#define ENC1 2  // Green 
#define ENC2 3  // Yellow

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
const int offsetA = 1;


Motor motor1(AIN1, AIN2,PWMA, offsetA, STBY ); // Adjusted constructor for library usage

void setup() {
  Serial.begin(9600);
  pinMode(ENC1,INPUT);
  pinMode(ENC2,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1), readEncoder, RISING);
  Serial.println("target pos");
}

void loop() {
  // set target position
  int target = 175*2;
  //int target = 350*sin(prevT/1e6);

  // PID constants
  float kp = 10;
  float kd = 0.25;
  float ki = 0.25;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  motor1.drive(pwr*dir);
  
  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();

}

void readEncoder() {
  int b = digitalRead(ENC2);

  if(b>0){
    posi++;
  }
  else{
    posi--;
  }
}