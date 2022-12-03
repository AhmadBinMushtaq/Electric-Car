#ifndef ACS712_H
#define ACS712_H

#pragma once
#include "Arduino.h"

class ACS712{

public:
    ACS712(int pin, double slope, double offset);

    double getCurrent();

private:
    int _pin;
    double _slope;
    double _offset;

};

#endif