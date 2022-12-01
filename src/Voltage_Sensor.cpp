#include <Voltage_Sensor.h>

Voltage_Sensor::Voltage_Sensor(int pin, int maxV){
    this->_pin = pin;
    this->_maxV = maxV;
}

double Voltage_Sensor::getVoltage(){
    return (double) (map(analogRead(this->_pin),0,4096,0,this->_maxV));
}