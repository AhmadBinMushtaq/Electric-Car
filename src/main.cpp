#include <Arduino.h>
#include <ESC.h>
#include <ACS712.h>
#include <Voltage_Sensor.h>
#include <Tachometer.h>

ESC motor1(1);
ESC motor2(1);
ESC motor3(1);
ESC motor4(1);

ACS712 acs1(1,1,1);
ACS712 acs2(1,1,1);
ACS712 acs3(1,1,1);
ACS712 acs4(1,1,1);

Voltage_Sensor voltage(1,1);

Tachometer IR(1);

void setup() {
    Serial.begin(115200);
}

void loop() {
    
    
}