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
  accAngleX = atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2)))*RAD_TO_DEG; //accAngleX = atan2(accX, accZ)*RAD_TO_DEG;  //  accAngleX = atan2(accY, accZ)*RAD_TO_DEG;
  accAngleY = atan(-accX / sqrt(pow(accY, 2) + pow(accZ, 2)))*RAD_TO_DEG; //accAngleY = atan2(accY, accZ)*RAD_TO_DEG; //   accAngleX = atan2(accY, accZ)*RAD_TO_DEG;
  accAngleZ = atan2(accY, accX)*RAD_TO_DEG; // atan(accY / sqrt(pow(accZ, 2) + pow(accX, 2)))*RAD_TO_DEG; // //  accAngleX = atan2(accY, accZ)*RAD_TO_DEG;

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
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}

/******************************************************************
* directionControl
******************************************************************/
void Motor::directionControl(float MotorIn[]) {

/* ---------- Control X-direction ----------*/
    if(accAngleX > 0){ //  accX
    if(accAngleX < 255){
      //Serial.println(accX);
      analogWrite(in2,accAngleX);
      analogWrite(enA, 255);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      //delay(2000);
    }
    else{
      //Serial.println("+255");
      analogWrite(in2,255);
      analogWrite(enA, 0);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }

    }
    if(accAngleX < 0){
    if(accAngleX > -255){
      //Serial.println(accX);
      analogWrite(in1, accAngleX-accAngleX-accAngleX);
      analogWrite(enA, 255);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else{
      analogWrite(enA, 0);
      //Serial.println("-255");
      analogWrite(in1, 255);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
    }

/* ---------- Control Y-direction ----------*/
    if(accAngleY > 0){ //  accY
    if(accAngleY < 255){
      analogWrite(enB, 255);
      //Serial.println(accY);
      analogWrite(in4,accAngleY);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      //delay(2000);
      }
    else{
      analogWrite(enB, 0);
      //Serial.println("+255");
      analogWrite(in4,255);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }

    }
    if(accAngleY < 0){
    if(accAngleY > -255){
      analogWrite(enB, 255);
      //Serial.println(accY);
      analogWrite(in3, accAngleY-accAngleY-accAngleY);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
    else{
      analogWrite(enB, 255);
      //Serial.println("-255");
      analogWrite(in3, 255);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }
    }
}

/******************************************************************
* printData
******************************************************************/
void Sensor::printData() {/* PRINT DATA */
  /* Se muestran las lecturas obtenidas con métodos distintos para determinar cual es preferible usar.
  Funcionan:    - ángulo de giro del acelerómetro
                - datos de aceleración, en g (1g = 9.81)
                - ángulo de giro obtenido usando el DMP integrado en el MPU6050
  No funcionan: - ángulo de giro del giroscopio
                  --> ángulo de giro con filtro High Pass y Low Pass */

// Llamar subrutinas, seguramente innecesario
  complementaryFilter();
  digitalmotionprocessor();

//raw data from accelerometer
//  Serial.print(" acc (RAW): ");
//  Serial.print (" X = ");
//  Serial.print( accX);
//  Serial.print (" Y = ");
//  Serial.print( accY);
//  Serial.print (" Z = ");
//  Serial.print( accZ);

// Mostrar lectura ángulo de giro del acelerómetro // FUNCIONA BIEN pero tiene los problemas relacionados con el "drift"
  Serial.println (" ");
  Serial.print(" Angle, accel (deg): ");
  Serial.print (" X = ");
  Serial.print( accAngleX);
  Serial.print (" Y = ");
  Serial.print( accAngleY);
  Serial.print (" Z = ");
  Serial.print( accAngleZ);

  Serial.println (" ");
// Mostrar lectura ángulo de giro del giroscopio // NO FUNCIONA da como output solo 0s, idk
  Serial.print(" Angle, gyro (deg): ");
  Serial.print (" X = ");
  Serial.print( gyroAngleX);
  Serial.print (" Y = ");
  Serial.print( gyroAngleY);
  Serial.print (" Z = ");
  Serial.print( gyroAngleZ);

  Serial.println (" ");
  Serial.println (" ");

// Mostrar lectura ángulo de giro filtrada usando void complementaryFilter // como depende del giroscopio la lectura no sale bien
  Serial.print(" Angle, filtered (deg): ");
  Serial.print (" X = ");
  Serial.print( currentAngleX);
  Serial.print (" Y = ");
  Serial.print( currentAngleY);
  Serial.print (" Z = ");
  Serial.print( currentAngleZ);

  Serial.println (" ");

// Mostar lectura aceleración (datos del ácelerómetro)  // FUNCIONA BIEN
  Serial.print(" Accel (g): ");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);

  Serial.println (" ");

  Serial.println (" ---------------------------------------------------------- ");
// Mostar los datos obtenidos usando el DMP integrado en el MPU6050 // TARDAN ENTRE 10-15s EN ESTABILIZARSE

// Mostrar Yaw, Pitch, Roll //  pitch y, roll x, yaw z // hay que confirmar que eje es que // SALE BIEN
    Serial.println (" ");
    Serial.print(" Angle, filtered (deg): ");
    // Serial.print (" ");
    Serial.print(" X ="); // Serial.print("\t");
    Serial.print(ypr[2] * 180/M_PI);
    Serial.print(" Y ="); //Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);;
    Serial.print(" Z ="); // Serial.print("ypr\t");
    Serial.println(ypr[0] * 180/M_PI);

    Serial.println (" ");

//   // Mostrar aceleracion // Puede que esté bien pero no se en qué unidades está
   Serial.print (" Acceleration:");
   //Serial.print("areal\t"); // Serial.print("areal\t");
   Serial.print(" X =");
   Serial.print(aaReal.x);
//   Serial.print("\t");
   Serial.print(" Y =");
   Serial.print(aaReal.y);
//   Serial.print("\t");
   Serial.print(" Z =");
   Serial.print(aaReal.z);

   Serial.println (" ");

   Serial.println("**************************************************************");

   delay(100);
}

/******************************************************************
* readSensor
******************************************************************/
void Sensor::readSensor() {
  setupMPU();
  mpu.getAcceleration(&accX, &accY, &accZ); //Acceleration from MPU
  //mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  dmp.dmpGetYawPitchRoll(); //Gyro from DMP
  recordAccelGryoRegisters();
  digitalmotionprocessor();
  //printData();

  gyroX = 0;
  gyroY = 0;
  gyroZ = 0;
  velX = 0;
  velY = 0;
  velZ = 0;
  accX = 0;
  accY = 0;
  accz = 0;

  statevector[9] = {gyroX, gyroY, gyroZ, velX, velY, velZ, accX, accY, accZ};

}
