#ifndef ACS712_H
#define ACS712_H

#pragma once
#include "Arduino.h"

class ACS712{

public:
    ACS712(byte pin, double slope, double offset);

    double getCurrent();

private:
    byte _pin;
    double _slope;
    double _offset;

};

#endif