#ifndef DEFINITION_H
#define DEFINITION_H

#pragma once
// Pin Definitions

#define RXPin1 2
#define RXPin2 4
#define RXPin3 5
#define RXPin4 18
#define RXPin5 19

#define VoltmeterPin 15
#define MaxVoltage 25

#define SteeringPin 23

#define ACS1Pin 35
#define ACS1Slope 9.04405
#define ACS1Offset -14.3177

#define ACS2Pin 34
#define ACS2Slope 9.04405
#define ACS2Offset -15.3177

#define ACS3Pin 26
#define ACS3Slope 22.3971
#define ACS3Offset -37.2174

#define ACS4Pin 25
#define ACS4Slope 22.3757
#define ACS4Offset -37.2776

#define BrakePin 33

#define SpeedometerPin 32
#define Circumference 455


#define MotorPin1 27
#define MotorPin2 14
#define MotorPin3 12
#define MotorPin4 13

//I2C Addresses

#define MPU6050Address 0x68

#define HMC5883LAddress 0x0D

#define BME280Address 0x76

#define RXMinDuty 1000
#define RXMaxDuty 2000

#define MotorMinDuty 1000
#define MotorMaxDuty 2000

#define SteeringMinDuty 500
#define SteeringMaxDuty 2500

#define BrakeMinDuty 1000
#define BrakeMaxDuty 2000

#define MaxSpeed 20000

#endif