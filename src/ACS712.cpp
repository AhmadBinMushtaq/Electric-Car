#include <ACS712.h>

ACS712::ACS712(byte pin, double slope, double offset)
{
    this->_pin = pin;
    this->_slope = slope;
    this->_offset = offset;
}
double ACS712::getCurrent()
{
    double current = (analogRead(this->_pin) * 3.3 / 4096 * this->_slope + this->_offset) * 10;
    return (constrain(current, 0, 200));
}