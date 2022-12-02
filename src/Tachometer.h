#ifndef TACHOMETER_H
#define TACHOMETER_H

#pragma once
#include "Arduino.h"

volatile unsigned long last_time = 0;
volatile double speed = 0;

void updateSpeed();

class Tachometer{

public:
    Tachometer(int);

    double getSpeed();

private:
    int _pin;

};

#endif