/*
===Hardware===
- Arduino Uno R3
- MPU-6050 /GY-521
- DC MOTOR
- L293D

==Connections==
-GND - GND
-VCC - VCC
-SDA - Pin A4
-SCL - Pin A5
-INT - Pin 2 // this pin allows for the use of DMP (Digital Motion Processor) onboard the MPU-6050 chip. This significantly simplifies the process of obtaining filtered rotation data from the sensor

===Software===
- Arduino IDE v1.8.42
- Arduino Wire library
- I2Cdev.h --> https://github.com/jrowberg/i2cdevlib
- MPU6050.h -->  https://github.com/jrowberg/i2cdevlib

/* COMMENT:
You have to edit the "MPU6050_6Axis_MotionApps20.h" file,
go to line:272-274 and you"ll find

 #ifndef MPU6050_DMP_FIFO_RATE_DIVISOR
 #define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01
 #endif

edit the 0x01 to 0x02 or 0x03. It will slow down the readings thus decreasing stress on your MCU.
*/

#include <MPU6050_6Axis_MotionApps_V6_12.h>               // esta libreria incluye las funciones para usar el DMP

/***********************  INCLUDE LIBRARIES ************************/
#include "SensorReadingsActuators.h"

MPU6050 mpu;

/******************************************************************/
/******************************************************************
* CONSTRUCTORS
******************************************************************/
/******************************************************************/
Sensor::Sensor(){


  /******************************************************************
  * MPU6050 setup
  ******************************************************************/
  Wire.begin();
  mpu.initialize();   // // The initialize( ) command sets the accelerometer to +/- 2g and the gyroscope to 250% per second by default. These are the most sensitive settings

/***********************  OFFSETS ************************/
  mpu.setXAccelOffset(-2842); // from calibration routine
  mpu.setYAccelOffset(-21); // from calibration routine
  mpu.setZAccelOffset(1088); // from calibration routine

  mpu.setXGyroOffset(25); // from calibration routine
  mpu.setYGyroOffset(-24); // from calibration routine
  mpu.setZGyroOffset(5); // from calibration routine


  /***********************   load and configure the DMP *************************/
  devStatus = mpu.dmpInitialize();      //  The dmpInitialize( ) command loads the firmware and configures it. It also initializes the FIFO buffer that’s going to
                                        // hold the combined data readings coming from the gyroscope and accelerometer. Providing everything has gone well with the initialization the DMP is enabled.

  // make sure it worked (returns 0 if so)
      if (devStatus == 0) {  // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      }

  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();

}

Actuator::Actuator() {

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);   // MOTOR X
  pinMode(in1, OUTPUT);   // MOTOR X
  pinMode(in2, OUTPUT);   // MOTOR X

  pinMode(enB, OUTPUT);   // MOTOR Y
  pinMode(in3, OUTPUT);   // MOTOR Y
  pinMode(in4, OUTPUT);   // MOTOR Y

  pinMode(enC, OUTPUT);   // MOTOR Z
  pinMode(in5, OUTPUT);   // MOTOR Z
  pinMode(in6, OUTPUT);   // MOTOR Z

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
}

/******************************************************************
* SENSOR FUNCTIONS
******************************************************************/

/******************************************************************
* readSensor
******************************************************************/
void Sensor::readSensor() {

  // Almacenar el estado anterior para derivar
  float angleX0 = statevector[0];
  float angleY0 = statevector[1];
  float angleZ0 = statevector[2];

  float velX0 = statevector[3];
  float velY0 = statevector[4];
  float velZ0 = statevector[5];

  // Definir Quaternion y vectores para las lecturas del DMP
  Quaternion q;           // [w, x, y, z]
  VectorInt16 aa;         // [x, y, z]
  VectorInt16 aaReal;     // [x, y, z]
  VectorInt16 aaWorld;    // [x, y, z]
  VectorFloat gravity;    // [x, y, z]
  float ypr[3];           // [yaw, pitch, roll]

  /***********************  Obtaining data (angleX, angleY, angleZ) ************************/
  while (fifoCount < packetSize) {fifoCount = mpu.getFIFOCount();}

  if (fifoCount == 1024) {
      mpu.resetFIFO();
  }
  else{
    if (fifoCount % packetSize != 0) {mpu.resetFIFO();}
      else{
          while (fifoCount >= packetSize) {
              mpu.getFIFOBytes(fifoBuffer,packetSize);
              fifoCount -= packetSize;
          }

          mpu.dmpGetQuaternion(&q,fifoBuffer);
          mpu.dmpGetGravity(&gravity,&q);
          mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);

          float angleZ = ypr[0] * 180/M_PI;     // yaw z
          float angleY = ypr[1] * 180/M_PI;     // pitch y
          float angleX = ypr[2] * 180/M_PI;     // roll x
        }
    }

  /***********************  Data for NN ************************/

  prevtime = currtime;
  currtime = micros();

  statevector[0] = angleX;
  statevector[1] = angleY;
  statevector[2] = angleZ;

  float velX = (angleX-angleX0)/((currtime-prevtime) * 1000000);
  float velY = (angleY-angleY0)/((currtime-prevtime) * 1000000);
  float velZ = (angleZ-angleZ0)/((currtime-prevtime) * 1000000);

  statevector[3] = velX;
  statevector[4] = velY;
  statevector[5] = velZ;

  float accX = (velX-velX0)/((currtime-prevtime) * 1000000);
  float accY = (velY-velY0)/((currtime-prevtime) * 1000000);
  float accZ = (velZ-velZ0)/((currtime-prevtime) * 1000000);

  statevector[6] = accX;
  statevector[7] = accY;
  statevector[8] = accZ;

}

/******************************************************************
* ACTUATOR FUNCTIONS
******************************************************************/

/******************************************************************
* directionControl
******************************************************************/
void Actuator::directionControl() {

  //Value between -255 and 255
  float Vx = MotorIn[0]; //in1 in2 enA
  float Vy = MotorIn[1]; //in3 in4 enB
  float Vz = MotorIn[2]; //in5 in6 enC

  /* ---------- Control X-direction ----------*/
  if (Vx > 0){
    analogWrite(enA, abs(Vx));
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(Vx < 0){
    analogWrite(enA, abs(Vx));
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if(Vx == 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  /* ---------- Control Y-direction ----------*/
  if (Vy > 0){
    analogWrite(enB, abs(Vy));
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(Vy < 0){
    analogWrite(enB, abs(Vy));
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if(Vy == 0){
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  /* ---------- Control Z-direction ----------*/
  if (Vz > 0){
    analogWrite(enC, abs(Vz));
    digitalWrite(in5, HIGH);
    digitalWrite(in6, LOW);
  }
  else if(Vz < 0){
    analogWrite(enC, abs(Vz));
    digitalWrite(in5, LOW);
    digitalWrite(in6, HIGH);
  }
  else if(Vz == 0){
    digitalWrite(in5, LOW);
    digitalWrite(in6, LOW);
  }

}
