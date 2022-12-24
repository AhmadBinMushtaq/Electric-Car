#include <Arduino.h>
#include <ESC.h>
#include <ACS712.h>
#include <Voltage_Sensor.h>
#include <GPS.h>


// Pin Definitions

#define RXPin1 2
#define RXPin2 4
#define RXPin3 5
#define RXPin4 18
#define RXPin5 19

#define VoltmeterPin 15
#define MaxVoltage 25

#define SteeringPin 23

#define ACS1Pin 35
#define ACS1Slope 22.3971
#define ACS1Offset -37.2174

#define ACS2Pin 34
#define ACS2Slope 22.3757
#define ACS2Offset -37.2776

#define ACS3Pin 26
#define ACS3Slope 9.04405
#define ACS3Offset -14.3177

#define ACS4Pin 25
#define ACS4Slope 9.04405
#define ACS4Offset -15.3177

#define BrakePin 33

#define SpeedometerPin 32
#define Circumference 455


#define MotorPin1 27
#define MotorPin2 14
#define MotorPin3 12
#define MotorPin4 13

//I2C Addresses

#define MPU6050Address 0x68

#define HMC5883LAddress 0x0D

#define BME280Address 0x76

#define RXMinDuty 1000
#define RXMaxDuty 2000

#define MotorMinDuty 1000
#define MotorMaxDuty 2000

#define SteeringMinDuty 500
#define SteeringMaxDuty 2500

#define BrakeMinDuty 1000
#define BrakeMaxDuty 2000

#define MaxSpeed 20000

volatile unsigned long last_time = 0;
volatile double speed = 0;
byte velocity = 0;

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

ESC motor1(MotorPin1, MotorMinDuty, MotorMaxDuty);
ESC motor2(MotorPin2, MotorMinDuty, MotorMaxDuty);
ESC motor3(MotorPin3, MotorMinDuty, MotorMaxDuty);
ESC motor4(MotorPin4, MotorMinDuty, MotorMaxDuty);
ACS712 acs1(ACS1Pin, ACS1Slope, ACS1Offset);
ACS712 acs2(ACS2Pin, ACS2Slope, ACS2Offset);
ACS712 acs3(ACS3Pin, ACS3Slope, ACS3Offset);
ACS712 acs4(ACS4Pin, ACS4Slope, ACS4Offset);
Voltage_Sensor voltage(VoltmeterPin, MaxVoltage);
ESC brake(BrakePin, BrakeMinDuty, BrakeMaxDuty);
ESC steering(SteeringPin, SteeringMinDuty, SteeringMaxDuty);

TaskHandle_t Task1;
TaskHandle_t Task2;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR SpeedInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
   if((long)(millis() - last_time) >= 2) {
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
    PulseWidth1 = map(constrain(pulses1, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }
  if (micros()-start_time1>30000){
    PulseWidth1 = 0;
  }

  if (pulses2 < 2100){
    PulseWidth2 = map(constrain(pulses2, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }
  if (micros()-start_time2>30000){
    PulseWidth2 = 0;
  }

  if (pulses3 < 2100){
    PulseWidth3 = map(constrain(pulses3, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }
  if (micros()-start_time3>30000){
    PulseWidth3 = 0;
  }

  if (pulses4 < 2100){
    PulseWidth4 = map(constrain(pulses4, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }
  if (micros()-start_time4>30000){
    PulseWidth4 = 0;
  }

  if (pulses5 < 2100){
    PulseWidth5 = map(constrain(pulses5, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, MaxSpeed);
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
    velocity = (byte) (speed*3.6);
  }
}

void updatePower(){
  Current1 = 0;
  Current2 = 0;
  Current3 = 0;
  Current4 = 0;
  Voltage = 0;

  for(int i = 0; i<10; i++){
    Current1 += acs1.getCurrent()/10;
    Current2 += acs2.getCurrent()/10;
    Current3 += acs3.getCurrent()/10;
    Current4 += acs4.getCurrent()/10;
    Voltage = voltage.getVoltage()/10;
  }

  Power = Voltage * (Current1+Current2+Current3+Current4);

}

void updateSpeed(){
  motorSpeed1 = (PulseWidth3*PulseWidth5/20000);
  motorSpeed2 = (PulseWidth3*PulseWidth5/20000);
  motorSpeed3 = (PulseWidth3*PulseWidth5/20000);
  motorSpeed4 = (PulseWidth3*PulseWidth5/20000);

  motor1.write(motorSpeed1);
  motor2.write(motorSpeed2);
  motor3.write(motorSpeed3);
  motor4.write(motorSpeed4);
  steering.write(PulseWidth1);

  // int velocity_d = PulseWidth3*PulseWidth5/100 - velocity;
  // int power_d = Power*velocity_d;
  // motorSpeed1 = constrain(motorSpeed1*(1+((Power/4)-(Current1*Voltage)+power_d/4)/(Current1*Voltage)), 0, 100);
  // motorSpeed2 = constrain(motorSpeed2*(1+((Power/4)-(Current2*Voltage)+power_d/4)/(Current2*Voltage)), 0, 100);
  // motorSpeed3 = constrain(motorSpeed3*(1+((Power/4)-(Current3*Voltage)+power_d/4)/(Current3*Voltage)), 0, 100);
  // motorSpeed4 = constrain(motorSpeed4*(1+((Power/4)-(Current4*Voltage)+power_d/4)/(Current4*Voltage)), 0, 100);

}

void attachInterrupt(){
  attachInterrupt(digitalPinToInterrupt(SpeedometerPin), SpeedInterrupt, RISING);
  // Serial.println("Speedometer Connected");
  attachInterrupt(digitalPinToInterrupt(RXPin1), RXInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin2), RXInterrupt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin3), RXInterrupt3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin4), RXInterrupt4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin5), RXInterrupt5, CHANGE);
  // Serial.println("Reciever channels connected");
}

void safeMode(){
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);
  bool triggered = false;
  long int trig_time = 0;

  for(;;){
    if(triggered){
      if(PulseWidth4<20 && PulseWidth3<5){
        if((millis()-trig_time) > 3000){
          // Serial.println("Vehicle Armed!!!");
          delay(200);
          return;
        }
      }
      else{
        triggered = false;
      }
    }
    if(!triggered){
      if(PulseWidth4<20 && PulseWidth3<5){
        triggered = true;
        trig_time = millis();
        // Serial.println("Triggered");
        delay(200);
      }
    }
  }

}

//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ){
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());
  attachInterrupt();
  delay(3000);


  for(;;){
    updateVelocity();
    Serial.write(velocity);
    // Serial.println();
    updateRX();
    // Serial.print(PulseWidth1);
    // Serial.print("  ");
    // Serial.print(PulseWidth2);
    // Serial.print("  ");
    // Serial.print(PulseWidth3);
    // Serial.print("  ");
    // Serial.print(PulseWidth4);
    // Serial.print("  ");
    // Serial.println(PulseWidth5);

    // while(Serial.available()){
    //   Serial.read();
    // }
    // Serial.write(PulseWidth1);
    // Serial.write("  ");
    // Serial.write(PulseWidth2);
    // Serial.write("  ");
    // Serial.write(PulseWidth3);
    // Serial.write("  ");
    // Serial.write(PulseWidth4);
    // Serial.write("  ");
    // Serial.write(PulseWidth5);
    // Serial.write("\n");

    delay(1);
  } 
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters ){
  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());
  // safeMode();

  for(;;){
    updatePower();

    // Serial.print(Current1, 1);
    // Serial.print("    ");
    // Serial.print(Current2, 1);
    // Serial.print("    ");
    // Serial.print(Current3, 1);
    // Serial.print("    ");
    // Serial.print(Current4, 1);
    // Serial.print("    ");

    updateSpeed();

    delay(10);

  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SpeedometerPin, INPUT_PULLDOWN);
  pinMode(RXPin1, INPUT_PULLDOWN);
  pinMode(RXPin2, INPUT_PULLDOWN);
  pinMode(RXPin3, INPUT_PULLDOWN);
  pinMode(RXPin4, INPUT_PULLDOWN);
  pinMode(RXPin5, INPUT_PULLDOWN);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

void loop() {
  
}