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
 */

/* NOTE!!!!!!!!!!
Se muestran las lecturas obtenidas con varios métodos para determinar cual es preferible usar. Los que no se usen se podrán borrar.
Funcionan:    - ángulo de giro del acelerómetro
              - datos de aceleración, en g (1g = 9.81 m/s^2)
              - ángulo de giro obtenido usando el DMP integrado en el MPU6050
No funcionan: - ángulo de giro del giroscopio
                --> ángulo de giro con filtro High Pass y Low Pass */

/***********************  INCLUDE LIBRARIES ************************/
#include <I2Cdev.h>
#include <math.h>
#include <MPU6050_6Axis_MotionApps20.h>               // esta libreria incluye las funciones para usar el DMP
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>                                 // incluir la libreria incluida de Arduino Wire.h de manera que no interfiera con MPU6050_6Axis_MotionApps20.h
#endif
#include <SensorReadingsActuators.h>
#include "Arduino.h"

/******************************************************************
* Constructors
******************************************************************/

Sensor::Sensor(){

  mpu_setup()


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
* Sensor functions
******************************************************************/

void mpu_setup()
{

  mpu.initialize();   // // The initialize( ) command sets the accelerometer to +/- 2g and the gyroscope to 250% per second by default. These are the most sensitive settings.

  // verify connection //  testConnection() will check that it can find the I2C device address associated with the IMU. This should be either 0x68 or 0x69.
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();      //  The dmpInitialize( ) command loads the firmware and configures it. It also initializes the FIFO buffer that’s going to
                                        // hold the combined data readings coming from the gyroscope and accelerometer. Providing everything has gone well with the initialization the DMP is enabled.

  // supply your own gyro offsets here, scaled for min sensitivity
  //  From a physics perspective the offsets provide a translation from the Body Frame to the Inertial Frame. The Body Frame is the IMU mounted on the robot, whereas the Inertial Frame is the
  // frame from which all angle and velocity calculations are done.
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
}


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

/******************************************************************
* IMU::readFifoBuffer_
******************************************************************/
void IMU::readFifoBuffer_() {
  // Clear the buffer so as we can get fresh values
  // The sensor is running a lot faster than our sample period
  mpu.resetFIFO();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
}

/******************************************************************
* readSensor
******************************************************************/
void Sensor::readSensor() {
  setupMPU();
  mpu.getAcceleration(&accX, &accY, &accZ); //Acceleration from MPU
  //dmp.dmpGetYawPitchRoll(); //Gyro from DMP
  recordAccelGryoRegisters();
  digitalmotionprocessor();

  float gX = 0;
  float gY = 0;
  float gZ = 0;
  float vX = 0;
  float vY = 0;
  float vZ = 0;
  float aX = gForceX;
  float aY = gForceY;
  float aZ = gForceZ;

  statevector[0] = gX;
  statevector[1] = gY;
  statevector[2] = gZ;
  statevector[3] = vX;
  statevector[4] = vY;
  statevector[5] = vZ;
  statevector[6] = aX;
  statevector[7] = aY;
  statevector[8] = aZ;

}




/******************************************************************
* Actuator functions
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
