#ifndef POWER_H
#define POWER_H

#include <ACS712.h>
#include <Voltage_Sensor.h>

class Power{
    int getPower(ACS712*, Voltage_Sensor*);
};


#endif