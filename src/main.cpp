#include <Arduino.h>
#include <ESC.h>

ESC motor1(12);
ESC motor2(13);
ESC motor3(14);
ESC motor4(15);


void setup() {
    Serial.begin(115200);
}

void loop() {
    for(int speed = 0; speed <= 100; speed++) {
        motor1.write(speed);
        motor2.write(speed);
        motor3.write(speed);
        motor4.write(speed);
        Serial.println(speed);
        delay(40);
    }
    delay(3000);

    for(int speed = 100; speed >= 0; speed--) {
        motor1.write(speed);
        motor2.write(speed);
        motor3.write(speed);
        motor4.write(speed);
        Serial.println(speed);
        delay(40);
    }
    delay(3000);
}