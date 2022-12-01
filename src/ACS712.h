#ifndef ACS712_H
#define ACS712_H

#pragma once
#include "Arduino.h"

class ACS712{

public:
    ACS712(int pin, int slope, int offset);

    double getCurrent();

private:
    int _pin;
    int _slope;
    int _offset;

};

#endif