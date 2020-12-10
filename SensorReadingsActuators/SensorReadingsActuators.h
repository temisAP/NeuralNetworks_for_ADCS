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

/******************************************************************
* Classes
******************************************************************/

class Sensor{

  public:
  /***********************  Define connections ************************/
  //DMP connection (Sensor) // El MPU6050 se conecta al pin 2 (digital) del Arduino para activar la funcionalidad del DMP
  const int INTERRUPT_PIN = 2;

  /***********************  Define parameters ************************/
  //const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
  //MPU6050 mpu(mpuAddress);
  MPU6050 mpu;

  //accelerations
  long accelX, accelY, accelZ;

  float gForceX, gForceY, gForceZ;
  int16_t accX, accY, accZ;

  float accAngleX, accAngleY, accAngleZ;

  //angular velocity
  float velX, velY, velZ; // double //gyroAngleX=0
  float currentVelX, currentVelY, currentVelZ, prevVelX=0, prevVelY=0, prevVelZ=0, error, prevError=0, errorSum=0; //

  //angle
  float gyroAngleX, gyroAngleY, gyroAngleZ; // double //gyroAngleX=0
  float currentAngleX, currentAngleY, currentAngleZ, prevAngleX=0, prevAngleY=0, prevAngleZ=0, error, prevError=0, errorSum=0; //

  //statevector

  float statevector[9];

  //time
  unsigned long currTime, prevTime=0, loopTime;
  float dt;

  // definir parámetros del DMP // no todos son necesarios
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  // definir Quaternion y vectores para las lecturas del DMP
  Quaternion q;           // [w, x, y, z]
  VectorInt16 aa;         // [x, y, z]
  VectorInt16 aaReal;     // [x, y, z]
  VectorInt16 aaWorld;    // [x, y, z]
  VectorFloat gravity;    // [x, y, z]
  float ypr[3];           // [yaw, pitch, roll]

  volatile bool mpuInterrupt = false;

  const float sampleTime = 0.005;

  /***********************  Define methods ************************/

  Sensor();
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
  * printData
  ******************************************************************/
  void printData();
  /******************************************************************
  * readSensor
  ******************************************************************/
  void readSensor();


};

class Actuator{

  /***********************  Motor connections ************************/
  // Motor X connections
  const int enA = 9;
  const int in1 = 8;
  const int in2 = 7;
  // Motor Y connections
  const int enB = 3;
  const int in3 = 5;
  const int in4 = 4;
  // Motor Z connections
  const int enC = 3;
  const int in5 = 5;
  const int in6 = 4;
  /***********************  Motor Methods ************************/
  Actuator();

  /******************************************************************
  * directionControl
  ******************************************************************/
  void directionControl(float);

};


#endif
