#include <Arduino.h>
#include <ESC.h>

static const int ESCPin1 = 13;
static const int ESCPin2 = 12;
static const int ESCPin3 = 14;
static const int ESCPin4 = 27;

ESC ESC1;
ESC ESC2;
ESC ESC3;
ESC ESC4;

void setup() {
    Serial.begin(115200);
    ESC1.attach(ESCPin1);
    ESC2.attach(ESCPin2);
    ESC3.attach(ESCPin3);
    ESC4.attach(ESCPin4);
}

void loop() {
    for(int speed = 0; speed <= 100; speed++) {
        ESC1.write(speed);
        ESC2.write(180-speed);
        ESC3.write(speed);
        ESC4.write(speed);
        Serial.println(speed);
        delay(40);
    }
    delay(3000);

    for(int speed = 100; speed >= 0; speed--) {
        ESC1.write(speed);
        ESC2.write(100-speed);
        ESC3.write(speed);
        ESC4.write(speed);
        Serial.println(speed);
        delay(40);
    }
    delay(3000);
}