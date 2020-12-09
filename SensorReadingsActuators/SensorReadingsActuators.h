/*
===Hardware===
- Arduino Uno R3
- MPU-6050 /GY-521
- DC MOTOR
- L293D

===Software===
- Arduino IDE v1.8.42
- Arduino Wire library
- I2Cdev.h --> https://github.com/jrowberg/i2cdevlib
- MPU6050.h -->  https://github.com/jrowberg/i2cdevlib
 */

#ifndef SensorReadingsActuators_h
#define SensorReadingsActuators_h

#include "Arduino.h"

class SensorReadingsActuators{

public:


/******************************************************************
*  dmpDataRead
******************************************************************/
void dmpDataReady();
/******************************************************************
* setupMPU
******************************************************************/
void setupMPU();
/******************************************************************
* recordAcceGyrolRegisters
******************************************************************/
void recordAccelGryoRegisters();
/******************************************************************
* digitalmotionprocessor
******************************************************************/
/* This method processes the data using the integrated DMP on the sensor, resulting in less processing done by the Arduino.
It is more precise than using the complementary filter according to some sources. */

void digitalmotionprocessor();

/******************************************************************
* directionControl
******************************************************************/
void directionControl(float);
/******************************************************************
* printData
******************************************************************/
void printData();
/******************************************************************
* readSensor
******************************************************************/
float (*readSensor)();

};

#endif
