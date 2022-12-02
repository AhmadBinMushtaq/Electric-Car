#include <Arduino.h>
#include <ESC.h>
#include <ACS712.h>
#include <Voltage_Sensor.h>
#include <Tachometer.h>

ESC motor1(12);
ESC motor2(13);
ESC motor3(14);
ESC motor4(15);

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