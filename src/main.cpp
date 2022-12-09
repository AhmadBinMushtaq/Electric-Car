#include <Arduino.h>
#include <ESC.h>
#include <ACS712.h>
#include <Voltage_Sensor.h>
#include <GPS.h>

#define RXPin1 19
#define RXPin2 18
#define RXPin3 5
#define RXPin4 4
#define RXPin5 2

#define VoltmeterPin 15
#define MaxVoltage 25

#define SteeringPin 23

#define ACS1Pin 36
#define ACS1Slope 9.04405
#define ACS1Offset -14.3177

#define ACS2Pin 39
#define ACS2Slope 9.04405
#define ACS2Offset -15.3177

#define ACS3Pin 34
#define ACS3Slope 9.04405
#define ACS3Offset -14.3177

#define ACS4Pin 35
#define ACS4Slope 9.04405
#define ACS4Offset -15.3177

#define BrakePin 32

#define SpeedometerPin 33
#define Circumference 455

#define LED1Pin 25

#define LED2Pin 26

#define MotorPin1 27
#define MotorPin2 14
#define MotorPin3 12
#define MotorPin4 13

#define MPU6050Address 0x68

#define HMC5883LAddress 0x7E

#define BME280Address 0x77

#define MinDuty 1000
#define MaxDuty 2000

#define MaxSpeed 20000

ESC motor1(MotorPin1);
ESC motor2(MotorPin2);
ESC motor3(MotorPin3);
ESC motor4(MotorPin4);

ACS712 acs1(ACS1Pin, ACS1Slope, ACS1Offset);
ACS712 acs2(ACS2Pin, ACS2Slope, ACS2Offset);
ACS712 acs3(ACS3Pin, ACS3Slope, ACS3Offset);
ACS712 acs4(ACS4Pin, ACS4Slope, ACS4Offset);

Voltage_Sensor voltage(VoltmeterPin, MaxVoltage);

ESC brake(BrakePin);

ESC steering(SteeringPin);  

volatile unsigned long last_time = 0;
volatile double speed = 0;
double velocity = 0;

volatile long start_time1 = 0;
volatile long current_time1 = 0;
volatile long pulses1 = 0;
int PulseWidth1 = 0;

volatile long start_time2 = 0;
volatile long current_time2 = 0;
volatile long pulses2 = 0;
int PulseWidth2 = 0;

volatile long start_time3 = 0;
volatile long current_time3 = 0;
volatile long pulses3 = 0;
int PulseWidth3 = 0;

volatile long start_time4 = 0;
volatile long current_time4 = 0;
volatile long pulses4 = 0;
int PulseWidth4 = 0;

volatile long start_time5 = 0;
volatile long current_time5 = 0;
volatile long pulses5 = 0;
int PulseWidth5 = 0;

int motorSpeed1 = 0;
int motorSpeed2 = 0;
int motorSpeed3 = 0;
int motorSpeed4 = 0;

double Current1 = 0;
double Current2 = 0;
double Current3 = 0;
double Current4 = 0;

double Voltage = 0;

double Power = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR SpeedInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
   if((long)(millis() - last_time) >= 5) {
    speed = Circumference/(millis() - last_time);
    last_time = millis();
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR RXInterrupt1() {
  portENTER_CRITICAL_ISR(&mux);

  current_time1 = micros();
  if (current_time1 > start_time1){
    pulses1 = current_time1 - start_time1;
    start_time1 = current_time1;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR RXInterrupt2() {
  portENTER_CRITICAL_ISR(&mux);

  current_time2 = micros();
  if (current_time2 > start_time2){
    pulses2 = current_time2 - start_time2;
    start_time2 = current_time2;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR RXInterrupt3() {
  portENTER_CRITICAL_ISR(&mux);

  current_time3 = micros();
  if (current_time3 > start_time3){
    pulses3 = current_time3 - start_time3;
    start_time3 = current_time3;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR RXInterrupt4() {
  portENTER_CRITICAL_ISR(&mux);

  current_time4 = micros();
  if (current_time4 > start_time4){
    pulses4 = current_time4 - start_time4;
    start_time4 = current_time4;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR RXInterrupt5() {
  portENTER_CRITICAL_ISR(&mux);

  current_time5 = micros();
  if (current_time5 > start_time5){
    pulses5 = current_time5 - start_time5;
    start_time5 = current_time5;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void updateRX(){
  if (pulses1 < 2100){
    PulseWidth1 = map(constrain(pulses1, MinDuty, MaxDuty), MinDuty, MaxDuty, 0, 100);
  }
  if (micros()-start_time1>30000){
    PulseWidth1 = 0;
  }

  if (pulses2 < 2100){
    PulseWidth2 = map(constrain(pulses2, MinDuty, MaxDuty), MinDuty, MaxDuty, 0, 100);
  }
  if (micros()-start_time2>30000){
    PulseWidth2 = 0;
  }

  if (pulses3 < 2100){
    PulseWidth3 = map(constrain(pulses3, MinDuty, MaxDuty), MinDuty, MaxDuty, 0, 100);
  }
  if (micros()-start_time3>30000){
    PulseWidth3 = 0;
  }

  if (pulses4 < 2100){
    PulseWidth4 = map(constrain(pulses4, MinDuty, MaxDuty), MinDuty, MaxDuty, 0, 100);
  }
  if (micros()-start_time4>30000){
    PulseWidth4 = 0;
  }

  if (pulses5 < 2100){
    PulseWidth5 = map(constrain(pulses5, MinDuty, MaxDuty), MinDuty, MaxDuty, 0, MaxSpeed);
  }
  if (micros()-start_time5>30000){
    PulseWidth5 = 0;
  }
}

void updateVelocity(){
  if(millis()-last_time>500){
    velocity = 0;
  }
  else{
    velocity = speed;
  }
}

void updatePower(){
  Current1 = acs1.getCurrent();
  Current2 = acs2.getCurrent();
  Current3 = acs3.getCurrent();
  Current4 = acs4.getCurrent();
  Voltage = voltage.getVoltage();
  Power = Voltage*(Current1+Current2+Current3+Current4);
}

void updateMotorSpeed(){
  int velocity_d = PulseWidth3*PulseWidth5/100 - velocity;
  int power_d = Power*velocity_d;
  motorSpeed1 = constrain(motorSpeed1*(1+((Power/4)-(Current1*Voltage)+power_d/4)/(Current1*Voltage)), 0, 100);
  motorSpeed2 = constrain(motorSpeed2*(1+((Power/4)-(Current2*Voltage)+power_d/4)/(Current2*Voltage)), 0, 100);
  motorSpeed3 = constrain(motorSpeed3*(1+((Power/4)-(Current3*Voltage)+power_d/4)/(Current3*Voltage)), 0, 100);
  motorSpeed4 = constrain(motorSpeed4*(1+((Power/4)-(Current4*Voltage)+power_d/4)/(Current4*Voltage)), 0, 100);
  motor1.write(motorSpeed1);
  motor2.write(motorSpeed2); 
  motor3.write(motorSpeed3);
  motor4.write(motorSpeed4);
}

void attachInterrupt(){
  attachInterrupt(digitalPinToInterrupt(SpeedometerPin), SpeedInterrupt, RISING);
  Serial.println("Speedometer Connected");
  attachInterrupt(digitalPinToInterrupt(RXPin1), RXInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin2), RXInterrupt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin3), RXInterrupt3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin4), RXInterrupt4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin5), RXInterrupt5, CHANGE);
  Serial.println("Reciever channels connected");
}

void setup() {

  Serial.begin(9600);
  Serial.println("Serial began on 9600 baud rate");
  pinMode(SpeedometerPin, INPUT_PULLDOWN);
  pinMode(RXPin1, INPUT_PULLDOWN);
  pinMode(RXPin2, INPUT_PULLDOWN);
  pinMode(RXPin3, INPUT_PULLDOWN);
  pinMode(RXPin4, INPUT_PULLDOWN);
  pinMode(RXPin5, INPUT_PULLDOWN);
  attachInterrupt();
  Serial.println("Interrupts Attached");
}

void loop() {
  updateRX();
  Serial.print(PulseWidth1);
  Serial.print("  ");
  Serial.print(PulseWidth2);
  Serial.print("  ");
  Serial.print(PulseWidth3);
  Serial.print("  ");
  Serial.print(PulseWidth4);
  Serial.print("  ");
  Serial.println(PulseWidth5);
  // updateVelocity();
  // updatePower();
  // updateMotorSpeed();
}