#include <Power.h>

int Power::getPower(ACS712* acs, Voltage_Sensor* voltage){
    return (int) acs->getCurrent() * voltage->getVoltage();
}