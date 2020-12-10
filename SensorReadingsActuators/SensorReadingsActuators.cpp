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
  //
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    //
  mpu.initialize();

/***********************  Define offsets ************************/
  mpu.setXAccelOffset(-2842); // from calibration sketch
  mpu.setYAccelOffset(-21); // from calibration sketch
  mpu.setZAccelOffset(1088); // from calibration sketch

  mpu.setXGyroOffset(25); // from calibration sketch
  mpu.setYGyroOffset(-24); // from calibration sketch
  mpu.setZGyroOffset(5); // from calibration sketch

  Serial.begin(115200);
  Wire.begin();

  // Iniciar MPU6050
 Serial.println(F("Initializing I2C devices..."));
 mpu.initialize();
 pinMode(INTERRUPT_PIN, INPUT);

 // Comprobar conexion
 Serial.println(F("Testing device connections..."));
 Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

 // Iniciar DMP
 Serial.println(F("Initializing DMP..."));
 devStatus = mpu.dmpInitialize();

 // Activar DMP
 if (devStatus == 0) {
     Serial.println(F("Enabling DMP..."));
     mpu.setDMPEnabled(true);

     // Activar interrupcion
     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady(), RISING);
     mpuIntStatus = mpu.getIntStatus();

     Serial.println(F("DMP ready! Waiting for first interrupt..."));
     dmpReady = true;

     // get expected DMP packet size for later comparison
     packetSize = mpu.dmpGetFIFOPacketSize();
 } else {
     // ERROR!
     // 1 = initial memory load failed
     // 2 = DMP configuration updates failed
     // (if it's going to break, usually the code will be 1)
     Serial.print(F("DMP Initialization failed (code "));
     Serial.print(devStatus);
     Serial.println(F(")"));
 }

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
*  dmpDataRead
******************************************************************/

void Sensor::dmpDataReady() {
    mpuInterrupt = true;
}

/******************************************************************
* setupMPU
******************************************************************/
void Sensor::setupMPU(){

// count time
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

// map accelerations
  accX = map(accX, -17000, 17000, -1500, 1500);
  accY = map(accY, -17000, 17000, -1500, 1500);
  accZ = map(accZ, -17000, 17000, -1500, 1500);
}

/******************************************************************
* recordAcceGyrolRegisters
******************************************************************/
void Sensor::recordAccelGryoRegisters() {
    // Calculate Angle of Inclination
/* atan2(y,z) function gives the angle in radians between the positive z-axis of a plane
and the point given by the coordinates (z,y) on that plane, with positive sign for
counter-clockwise angles (right half-plane, y > 0), and negative sign for clockwise
angles (left half-plane, y < 0). */
  //accAngleX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2)))*RAD_TO_DEG; //accAngleX = atan2(accX, accZ)*RAD_TO_DEG;  //  accAngleX = atan2(accY, accZ)*RAD_TO_DEG;
  //accAngleY = atan(-accX / sqrt(pow(accY, 2) + pow(accZ, 2)))*RAD_TO_DEG; //accAngleY = atan2(accY, accZ)*RAD_TO_DEG; //   accAngleX = atan2(accY, accZ)*RAD_TO_DEG;
  //accAngleZ = atan2(accY, accX)*RAD_TO_DEG; // atan(accY / sqrt(pow(accZ, 2) + pow(accX, 2)))*RAD_TO_DEG; // //  accAngleX = atan2(accY, accZ)*RAD_TO_DEG;

  // Calculate Acceleration
  gForceX = accX / 16388.0;  // 16388 is the value that the accel would show when subject to 1g acceleration
  gForceY = accY / 16388.0;
  gForceZ = accZ / 16388.0;

}

/******************************************************************
* digitalmotionprocessor
******************************************************************/
/* This method processes the data using the integrated DMP on the sensor, resulting in less processing done by the Arduino.
It is more precise than using the complementary filter according to some sources. */

void Sensor::digitalmotionprocessor() {
    if (!dmpReady) return;

    // Ejecutar mientras no hay interrupcion
    while (!mpuInterrupt && fifoCount < packetSize) {
        // AQUI EL RESTO DEL CODIGO DE TU PROGRRAMA
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Obtener datos del FIFO
    fifoCount = mpu.getFIFOCount();

    // Controlar overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
}

    // Yaw, Pitch, Roll //  pitch y, roll x, yaw z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Mostrar aceleracion
    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetAccel(&aa, fifoBuffer);
    //mpu.dmpGetGravity(&gravity, &q);
    //mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}

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
