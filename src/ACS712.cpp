#include <ACS712.h>

ACS712::ACS712(int pin, double slope, double offset){
    this->_pin = pin;
    this->_slope = slope;
    this->_offset = offset;
}

double ACS712::getCurrent(){
    return (double) ((analogRead(this->_pin)*3.3/4096) * this->_slope + this->_offset);
}