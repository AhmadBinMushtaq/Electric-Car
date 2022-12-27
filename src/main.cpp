//Include required libraries

//Arduino header file 
//to use the arduino specific functions.
#include <Arduino.h>
//Customized ESC library 
//to output speed signal to ESC as a PWM.
#include <ESC.h>
//Customized Current Sensor library 
//to get the value of current from sensors.
#include <ACS712.h>
//Customized voltage sensor library 
//to get values of voltage from voltage sensor.
#include <Voltage_Sensor.h>
//Customized GPS library 
//to get longitude and latitude values from GPS module.
#include <GPS.h>


// Pin Definitions
//Flysky Reciever pins
#define RXPin1 2
#define RXPin2 4
#define RXPin3 5
#define RXPin4 18
#define RXPin5 19

//0-25V Voltmeter pin
#define VoltmeterPin 15
//Max voltage of voltmeter
#define MaxVoltage 25

//Servo pin for steering
#define SteeringPin 23

//Pins for current sensors
#define ACS1Pin 35
#define ACS2Pin 34
#define ACS3Pin 26
#define ACS4Pin 25

//Slopes and offset of current sensors
//for calculating the current from input analog values
#define ACS1Slope 22.3971
#define ACS1Offset -37.2174
#define ACS2Slope 22.3757
#define ACS2Offset -37.2776
#define ACS3Slope 9.04405
#define ACS3Offset -14.3177
#define ACS4Slope 9.04405
#define ACS4Offset -15.3177

//Pin connected to brake servo
#define BrakePin 33

//Pin connected to IR Tachometer
#define SpeedometerPin 32
#define Circumference 455

//Pins connected to Electronic Speed controller
//of all the BLDC motors
#define MotorPin1 27
#define MotorPin2 14
#define MotorPin3 12
#define MotorPin4 13

//I2C Addresses
//for different sensors
//MPU6050 address. (Triple axis Accelerometer + Gyroscope)
#define MPU6050Address 0x68
//HMC5883L address. (Triple axis Compass/Magnetometer)
#define HMC5883LAddress 0x0D
//BME280 address. (Environment Sensor. Temperature + Humidity + Altitude + Pressure)
#define BME280Address 0x76

//Define Constant values for processing
//Min. Duty Cycle for PWM input of receiver
#define RXMinDuty 1000
//Max. Duty Cycle for PWM input of receiver
#define RXMaxDuty 2000

//Min. Duty Cycle for PWM input of ESC for BLDC motors
#define MotorMinDuty 1000
//Max. Duty Cycle for PWM input of ESC for BLDC motors
#define MotorMaxDuty 2000

//Min. Duty Cycle for PWM input of Steering Servo
#define SteeringMinDuty 500
//Max. Duty Cycle for PWM input of Steering Servo
#define SteeringMaxDuty 2500

//Min. Duty Cycle for PWM input of Brake Servo
#define BrakeMinDuty 1000
//Max. Duty Cycle for PWM input of Brake Servo
#define BrakeMaxDuty 2000

//Declare Interrupt variables

//stores millis for the last interrupt recorded from speedometer
volatile unsigned long last_time = 0;

//Temporarily store the value of speed in this variable
volatile double speed = 0;

//Permanent variable for speed. Updated in UpdateSpeed function
int velocity = 0;

//Rising Edge time micros of channel 1 PWM interrupt from Flysky reciever
volatile long start_time1 = 0;
//New interrupt time micros of channel 1 PWM interrupt from Flysky reciever
volatile long current_time1 = 0;
//Duration of PWM pulse input
volatile long pulses1 = 0;
//PulseWidth in percentage
int PulseWidth1 = 0;

//Rising Edge time micros of channel 2 PWM interrupt from Flysky reciever
volatile long start_time2 = 0;
//New interrupt time micros of channel 2 PWM interrupt from Flysky reciever
volatile long current_time2 = 0;
//Duration of PWM pulse input
volatile long pulses2 = 0;
//PulseWidth in percentage
int PulseWidth2 = 0;

//Rising Edge time micros of channel 3 PWM interrupt from Flysky reciever
volatile long start_time3 = 0;
//New interrupt time micros of channel 3 PWM interrupt from Flysky reciever
volatile long current_time3 = 0;
//Duration of PWM pulse input
volatile long pulses3 = 0;
//PulseWidth in percentage
int PulseWidth3 = 0;

//Rising Edge time micros of channel 4 PWM interrupt from Flysky reciever
volatile long start_time4 = 0;
//New interrupt time micros of channel 4 PWM interrupt from Flysky reciever
volatile long current_time4 = 0;
//Duration of PWM pulse input
volatile long pulses4 = 0;
//PulseWidth in percentage
int PulseWidth4 = 0;

//Rising Edge time micros of channel 5 PWM interrupt from Flysky reciever
volatile long start_time5 = 0;
//New interrupt time micros of channel 5 PWM interrupt from Flysky reciever
volatile long current_time5 = 0;
//Duration of PWM pulse input
volatile long pulses5 = 0;
//PulseWidth in percentage
int PulseWidth5 = 0;

//Declare remaining variables

//Output speed of motors
int motorSpeed = 0;

//Total Current
int Current = 0;

//Current through each sensor
double Current1 = 0;
double Current2 = 0;
double Current3 = 0;
double Current4 = 0;

//Voltage reading from sensor
double Voltage = 0;

//Total Power
int Power = 0;

//Create 4 objects of ESC class for Electronic Speed control of BLDC motors
ESC motor1(MotorPin1, MotorMinDuty, MotorMaxDuty);
ESC motor2(MotorPin2, MotorMinDuty, MotorMaxDuty);
ESC motor3(MotorPin3, MotorMinDuty, MotorMaxDuty);
ESC motor4(MotorPin4, MotorMinDuty, MotorMaxDuty);

//Create 4 objects of ACS712 class for 30A Current Sensors
ACS712 acs1(ACS1Pin, ACS1Slope, ACS1Offset);
ACS712 acs2(ACS2Pin, ACS2Slope, ACS2Offset);
ACS712 acs3(ACS3Pin, ACS3Slope, ACS3Offset);
ACS712 acs4(ACS4Pin, ACS4Slope, ACS4Offset);

//Create an object of Voltage_Sensor class for 0-25V voltage sensor
Voltage_Sensor voltage(VoltmeterPin, MaxVoltage);

//Object of ESC class for servo motors in brakes and steering
ESC brake(BrakePin, BrakeMinDuty, BrakeMaxDuty);
ESC steering(SteeringPin, SteeringMinDuty, SteeringMaxDuty);


//Create two tasks for separate cores of Xtensa LX6
TaskHandle_t Task1;
TaskHandle_t Task2;


//Interrupt functions
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//Speedometer interrupt
void IRAM_ATTR SpeedInterrupt() {
  portENTER_CRITICAL_ISR(&mux);

  //Ignore any glitches in <2ms
   if((long)(millis() - last_time) >= 2) {
    speed = Circumference/(millis() - last_time);
    last_time = millis();
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// Flysky reciever Channel 1 interrupt
void IRAM_ATTR RXInterrupt1() {
  portENTER_CRITICAL_ISR(&mux);

  current_time1 = micros();
  if (current_time1 > start_time1){
    pulses1 = current_time1 - start_time1;
    start_time1 = current_time1;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

//Flysky Reciever channel 2 interrupt
void IRAM_ATTR RXInterrupt2() {
  portENTER_CRITICAL_ISR(&mux);

  current_time2 = micros();
  if (current_time2 > start_time2){
    pulses2 = current_time2 - start_time2;
    start_time2 = current_time2;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

//Flysky Reciever channel 3 interrupt
void IRAM_ATTR RXInterrupt3() {
  portENTER_CRITICAL_ISR(&mux);

  current_time3 = micros();
  if (current_time3 > start_time3){
    pulses3 = current_time3 - start_time3;
    start_time3 = current_time3;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

//Flysky reciever Channel 4 interrupt
void IRAM_ATTR RXInterrupt4() {
  portENTER_CRITICAL_ISR(&mux);

  current_time4 = micros();
  if (current_time4 > start_time4){
    pulses4 = current_time4 - start_time4;
    start_time4 = current_time4;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

//Flysky Reciever channel 5 interrupt
void IRAM_ATTR RXInterrupt5() {
  portENTER_CRITICAL_ISR(&mux);

  current_time5 = micros();
  if (current_time5 > start_time5){
    pulses5 = current_time5 - start_time5;
    start_time5 = current_time5;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

//Store the Reciever inputs from volatile variables to permanent variables
void updateRX(){

  //Check if the pulsewidth1 is valid
  if (pulses1 < 2100){
    //Map pulse 1 to Percentage pulsewidth
    PulseWidth1 = map(constrain(pulses1, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }

  //If pulsewidth 1 is invalid
  if (micros()-start_time1>30000){
    //Reset value of pulsewidth1
    PulseWidth1 = 0;
  }

  //Check if the pulsewidth2 is valid
  if (pulses2 < 2100){
    //Map pulse 2 to percentage pulsewidth
    PulseWidth2 = map(constrain(pulses2, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }

  //If pulsewidth 2 is invalid
  if (micros()-start_time2>30000){
    //Reset value of pulsewidth2
    PulseWidth2 = 0;
  }

  //Check if pulsewidth3 is valid
  if (pulses3 < 2100){
    //Map pulse 3 to percentage pulsewidth
    PulseWidth3 = map(constrain(pulses3, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }

  //If pulsewidth 3 is invalid
  if (micros()-start_time3>30000){
    //Reset value of pulsewidth3
    PulseWidth3 = 0;
  }

  //Check if pulsewidth4 is valid
  if (pulses4 < 2100){
    //Map pulse 4 to percentage pulsewidth
    PulseWidth4 = map(constrain(pulses4, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }
  //If pulsewidth 4 is invalid
  if (micros()-start_time4>30000){
    //Reset value of pulsewidth4
    PulseWidth4 = 0;
  }

  //Check if pulsewidth5 is valid
  if (pulses5 < 2100){
    //Map pulse 5 to percentage pulsewidth
    PulseWidth5 = map(constrain(pulses5, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, 0, 100);
  }
  //If pulsewidth5 is invalid
  if (micros()-start_time5>30000){
    //Reset value of pulsewidth5
    PulseWidth5 = 0;
  }
}

//Store the value of speed from volatile variable to permanent variable for processing
void updateVelocity(){
  //Check if the axle is stopped
  if(millis()-last_time>500){
    //Reset velocity to zero
    velocity = 0;
  }
  //Check if any glitches in speedometer values
  else if(velocity-speed > 5){
    //Smooth out the glitches
    velocity -= 5;
  }
  //Check for glitches in negative direction
  else if(speed-velocity>5){
    //Again smooth out the glitches
    velocity += 5;
  }
  //If no glitches
  else{
    //Store velocity to permanent variable
    velocity = speed;
  }
}

//Update the values of Voltage, Current and Power by updating Current & voltage sensor and adding the formula for power

void updatePower(){
  // Current1 = 0;
  // Current2 = 0;
  // Current3 = 0;
  // Current4 = 0;
  // Voltage = 0;

  // for(int i = 0; i<10; i++){
  //   Current1 += acs1.getCurrent()/10;
  //   Current2 += acs2.getCurrent()/10;
  //   Current3 += acs3.getCurrent()/10;
  //   Current4 += acs4.getCurrent()/10;
  //   Voltage = voltage.getVoltage()/10;
  // }

  //Store values of currents and voltages to temporary variables
  int current1 = acs1.getCurrent();
  int current2 = acs2.getCurrent();
  int current3 = acs3.getCurrent();
  int current4 = acs4.getCurrent();
  Voltage = voltage.getVoltage();

  //Check for glitches
  if(Current1 - current1 > 5){
    //Smooth out the glitches
    Current1 -=5;
  } 
  //Check for glitches in negative direction
  else if(current1 - Current1 > 5){
    //Again smooth out glitches
    Current1 +=5;
  }
  //If no glitches 
  else{
    //Store current1 to permanent variable
    Current1 = current1;
  }

  //Check for glitches
  if(Current2 - current2 > 5){
    //Smooth out glitches
    Current2 -=5;
  } 
  //Check for glitches in negative direction
  else if(current2 - Current2 > 5){
    //Again smooth out glitches
    Current2 +=5;
  }
  //If no glitches 
  else{
    //Store current2 to permanent variable
    Current2 = current2;
  }

  //Check for glitches
  if(Current3 - current3 > 5){
    //Smooth out glitches
    Current3 -=5;
  } 
  //Check for glitches in negative direction
  else if(current3 - Current3 > 5){
    //Again smooth out glitches
    Current3 +=5;
  } 
  //If no glitches
  else{
    //Store current3 to permanent variable
    Current3 = current3;
  }

  //Check for glitches
  if(Current4 - current4 > 5){
    //Smooth out glitches
    Current4 -=5;
  } 
  //Check for glitches in negative direction
  else if(current4 - Current4 > 5){
    //Again smooth out glitches
    Current4 +=5;
  } 
  //If no glitches
  else{
    //Store current4 to permanent variable
    Current4 = current4;
  }

  //Update total current
  Current = Current1+Current2+Current3+Current4;
  //Update total power
  Power = (int)(Voltage*Current/10);

}

//Update the signal going towards BLDC ESC to update speed of each motor.
//Update pulsewidth going to steering pin to update direction of steering.
void updateSpeed(){
  motorSpeed = PulseWidth3;

  motor1.write(motorSpeed);
  motor2.write(motorSpeed);
  motor3.write(motorSpeed);
  motor4.write(motorSpeed);

  // Serial.print(motorSpeed);
  // Serial.print("    ");
  // Serial.println(velocity);
  steering.write(PulseWidth1);

  // int velocity_d = PulseWidth3*PulseWidth5/100 - velocity;
  // int power_d = Power*velocity_d;
  // motorSpeed1 = constrain(motorSpeed1*(1+((Power/4)-(Current1*Voltage)+power_d/4)/(Current1*Voltage)), 0, 100);
  // motorSpeed2 = constrain(motorSpeed2*(1+((Power/4)-(Current2*Voltage)+power_d/4)/(Current2*Voltage)), 0, 100);
  // motorSpeed3 = constrain(motorSpeed3*(1+((Power/4)-(Current3*Voltage)+power_d/4)/(Current3*Voltage)), 0, 100);
  // motorSpeed4 = constrain(motorSpeed4*(1+((Power/4)-(Current4*Voltage)+power_d/4)/(Current4*Voltage)), 0, 100);

}

//Attach interrupts to their respective functions
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

//Safe Mode: When rebooted, the vehicle will be disarmed until specified inputs are recieved from the
//Remote control in order to ensure the safety and avoid glitches in car performance
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

// Sends data to HC12 for real-time data plotting
void send(){
    Serial.print(velocity);
    Serial.print(",");
    Serial.print(Current);
    Serial.print(",");
    Serial.print(Power);
    Serial.print(",");
    Serial.print(motorSpeed);
    Serial.print(",");
    Serial.print(PulseWidth1);
    Serial.println("");

}

//Task 1 pinned to core0. Mainly for communication interrupts of reciever and speedometer.
void Task1code( void * pvParameters ){
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());
  attachInterrupt();
  delay(3000);


  for(;;){
    updateVelocity();

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

    delay(10);
  } 
}

//Task2code pinned to core1. Mainly for control loop and outputs.
void Task2code( void * pvParameters ){
  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());
  // safeMode();

  for(;;){
    updatePower();

    // Serial.println(Voltage);

    // Serial.print(Current1,1);
    // Serial.print("    ");
    // Serial.print(Current2,1);
    // Serial.print("    ");
    // Serial.print(Current3,1);
    // Serial.print("    ");
    // Serial.print(Current4,1);
    // Serial.print("    ");
    // Serial.print((byte)Current1);
    // Serial.print("    ");
    // Serial.print((byte)Current2);
    // Serial.print("    ");
    // Serial.print((byte)Current3);
    // Serial.print("    ");
    // Serial.print((byte)Current4);
    // Serial.println("    ");

    send();

    updateSpeed();

    delay(100);

  }
}

//Setup code to set the pinModes and pin the tasks to specific cores when ESP32 reboots.
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


//Empty loop function.
void loop() {
  
}