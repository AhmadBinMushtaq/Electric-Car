#ifndef VOLTAGE_SENSOR_H
#define VOLTAGE_SENSOR_H

#pragma once
#include "Arduino.h"

class Voltage_Sensor{

public:
    Voltage_Sensor(int, int);
    double getVoltage();

private:
    int _pin;
    int _maxV;
};

#endif