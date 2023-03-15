//Include required libraries

//Arduino header file 
//to use the arduino specific functions.
#include <Arduino.h>
//Customized ESC library 
//to output speed signal to ESC as a PWM.
#include <ESC.h>
//Customized Current Sensor library 
//to get the value of current from sensors.



// Pin Definitions
//Flysky Reciever pins
#define RXPin1 18
#define RXPin2 19

#define MotorPin1 13
#define MotorPin2 35
#define MotorPin3 33
#define MotorPin4 14

//Define Constant values for processing
//Min. Duty Cycle for PWM input of receiver
#define RXMinDuty 1000
//Max. Duty Cycle for PWM input of receiver
#define RXMaxDuty 2000

//Min. Duty Cycle for PWM input of ESC for BLDC motors
#define MotorMinDuty 0
//Max. Duty Cycle for PWM input of ESC for BLDC motors
#define MotorMaxDuty 20000

//Declare Interrupt variables

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

//Create 4 objects of ESC class for Electronic Speed control of BLDC motors
ESC motor1(MotorPin1, MotorMinDuty, MotorMaxDuty);
ESC motor2(MotorPin2, MotorMinDuty, MotorMaxDuty);
ESC motor3(MotorPin3, MotorMinDuty, MotorMaxDuty);
ESC motor4(MotorPin4, MotorMinDuty, MotorMaxDuty);

//Create two tasks for separate cores of Xtensa LX6
TaskHandle_t Task1;
TaskHandle_t Task2;


//Interrupt functions
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

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
    //Map pulse 1 to Percentage pulsewidth
    PulseWidth2 = map(constrain(pulses2, RXMinDuty, RXMaxDuty), RXMinDuty, RXMaxDuty, -100, 100);
  }

  //If pulsewidth 1 is invalid
  if (micros()-start_time2>30000){
    //Reset value of pulsewidth2
    PulseWidth2 = 0;
  }
}

//Update the signal going towards BLDC ESC to update speed of each motor.
//Update pulsewidth going to steering pin to update direction of steering.
void updateSpeed(){
  motor1.write(PulseWidth1);
  motor2.write(PulseWidth1);
  motor3.write(PulseWidth1);
  motor4.write(PulseWidth1);

}

//Attach interrupts to their respective functions
void attachInterrupt(){
  attachInterrupt(digitalPinToInterrupt(RXPin1), RXInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RXPin2), RXInterrupt2, CHANGE);
}

//Task 1 pinned to core0. Mainly for communication interrupts of reciever and speedometer.
void Task1code( void * pvParameters ){
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());
  attachInterrupt();
  delay(3000);


  for(;;){
    updateRX();
    Serial.print(PulseWidth1);
    Serial.print("  ");
    Serial.print(PulseWidth2);
    Serial.println();
    delay(10);
  } 
}

//Task2code pinned to core1. Mainly for control loop and outputs.
void Task2code( void * pvParameters ){
  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());
  // safeMode();

  for(;;){
    updateSpeed();

    delay(100);

  }
}

//Setup code to set the pinModes and pin the tasks to specific cores when ESP32 reboots.
void setup() {
  Serial.begin(9600);
  pinMode(RXPin1, INPUT_PULLDOWN);
  pinMode(RXPin2, INPUT_PULLDOWN);
  digitalWrite(32, LOW);
  digitalWrite(34, LOW);
  digitalWrite(27, LOW);
  digitalWrite(12, LOW);


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