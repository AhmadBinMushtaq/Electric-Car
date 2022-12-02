#include <Tachometer.h>

Tachometer::Tachometer(int pin){
    this->_pin = pin;
    attachInterrupt(pin, updateSpeed, FALLING);
}

void updateSpeed(){
    speed = 1/(millis()-last_time);
    last_time = millis();
}

double Tachometer::getSpeed(){
    return speed;
}