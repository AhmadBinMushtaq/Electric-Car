#include <Arduino.h>
#include <ESC.h>
#include <ACS712.h>
#include <Voltage_Sensor.h>

// ESC motor1(2);
// ESC motor2(1);
// ESC motor3(1);
// ESC motor4(1);

// ACS712 acs1(2,9.04405, -14.3177);
// ACS712 acs2(1,9.04405, -15.3177);
// ACS712 acs3(1,16.7752,-28.1059);

// Voltage_Sensor voltage(2,25);

volatile unsigned long last_time = 0;
volatile double speed = 0;

 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
   if((long)(millis() - last_time) >= 20) {
    speed = 1000/(millis()-last_time);
    last_time = millis();
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void setup() {
    Serial.begin(115200);
    pinMode(2, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(2), handleInterrupt, FALLING);
}

void loop() {
    Serial.println(speed);
}