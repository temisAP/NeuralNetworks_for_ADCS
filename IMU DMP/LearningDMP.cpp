//%%%%%%%%%%%%%%%%%%%%%%%%%% Notes about the MPU6050 IMU and the integrated DMP for extracting useful data  %%%%%%%%%%%%%%%%%%%%%%%%%%//
// Original Source: https://mjwhite8119.github.io/Robots/mpu6050

/////////////////////////// DEVICE SENSITIVITY ///////////////////////////

/* - The analog voltage read from the capacitive sensors is converted to digital signal in the range of 0 to 32750 values.
   -These values make up the measurement units for the gyroscope and accelerometer. The measurement units have to be split up to represent meaningful information.
   - The MPU6050 allocates its measurement units by creating four sensitivity levels, as shown in the slide below. The sensitive level that you chose depends on how you’re going to use the IMU.
   - For instance, if the robot is going to do high speed rotations of over 1000° per second (167 RPM) then you should set the gyro sensitivity to 2000°. In this case, since the gyro has to
    cover a lot of rotational ground in a very short period time it needs to split up its measurement units sparingly.

Gyro measurement units = 32 750                                                 accelerometer measurement units = 32 750
32 750 / degrees = sensitivity                                                  32 750 / g = sensitivity
i.e. 32 750 / 250 = 131 measurement units / degree                              i.e. 32 750 / 2 = 16 384 measurement units / g          */

#include <I2Cdev.h>
#include <math.h>
#include <MPU6050_6Axis_MotionApps20.h>               // esta libreria incluye las funciones para usar el DMP
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>                                 // incluir la libreria incluida de Arduino Wire.h de manera que no interfiera con MPU6050_6Axis_MotionApps20.h
#endif
#include <SensorReadingsActuators.h>



void mpu_setup()
{

  mpu.initialize();   // // The initialize( ) command sets the accelerometer to +/- 2g and the gyroscope to 250% per second by default. These are the most sensitive settings.
  pinMode(INTERRUPT_PIN, INPUT);

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
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

// The next part of the code that sets up an interrupt was a little confusing at first, since I’d seen many wiring diagrams that don’t show the interrupt pin connected.
//Turns out that you can use the MPU6050 both with and without the interrupt. If the IMU is being used in projects that require you to send control actions to the robot then
//it’s probably not desirable to use the interrupt method, since it may be difficult to get the control actions through. At the end of the article I’ll show you how to use MPU6050
//without the interrupt (in polling mode). The interrupt just calls a short ISR that sets a flag when the MPU’s data buffer is full. The ISR is shown at the end of the above code sample.
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

/* To get the pitch angle using the raw sensor data there are several translations that need to be made.

    The gyroscope gives the rate of change of the angular position over time, called angular velocity. Therefore, it has to be integrated over a time to get the angle position.

    Before making any calculations you have to translate readings from the body frame to the inertial frame by using the IMU offsets.

    You need to convert the raw IMU sensor readings into degrees.

    To mitigate the effects of drift you need to combine gyro information with readings coming from the accelerometer. */

/* Gyroscopes do NOT report angles, they report the speed at which the device is turning, or angular velocity. In order to get the angle position you have to integrate it over time.
The code below shows the time integration being applied to the angular rate information coming from the gyroscope. */


/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% METHOD 1: CONVERTING RAW VALUES (NOT THE ONE WE ARE USING) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/*The raw sensor readings need to be converted into degrees prior to applying the integration. This is done by dividing the readings by the sensitivity measurement unit.
Recall that at the highest sensitivity setting each degree is represented by 131 measurement units. */

//--------------------------------------------//
// Get angle from the gyroscope. Uint: degree
//--------------------------------------------//
float IMU::getGyroRoll(int gyroX, int gyroBiasX, uint32_t lastTime)
{
  float gyroRoll;

  //integrate gyroscope value in order to get angle value
  gyroRoll = ((gyroX - gyroBiasX ) / 131) * ((float)(micros() - lastTime) / 1000000);
  return gyroRoll;
}

/*The following code shows how you would calculate the pitch angle coming from the accelerometer. The formulas for computing the angles for yaw, pitch, and roll can be found online.
Again, you need to subtract the offset values. The resulting output is in radians, which need to be converted into degrees. */

//-----------------------------------------------//
//Get angle from the accelerometer. Uint: degree
//-----------------------------------------------//
float IMU::getAccRoll(int accY, int accBiasY, int accZ, int accBiasZ)
{
  float accRoll;

  //calculate angle value
  accRoll = (atan2((accY - accBiasY), (accZ - accBiasZ))) * RAD_TO_DEG;

  if (accRoll <= 360 && accRoll >= 180) {
    accRoll = 360 - accRoll;
  }
  return accRoll;
}

/*After computing the pitch angles coming from the accelerometer and the gyroscope a Complementary Filter is used to the mitigate the vibration effects that the accelerometer is subjected to and,
more importantly, the long term drift effects of the gyroscope.

So where does drift come from? As just mentioned, gyroscopes do not report angles, they report the speed at which the device is turning. In order to get the angle position you have to integrate
it over time. You may remember from your calculus class that to get position you have to integrate velocity. Since the time period used on a computer has some defined length, like 10 milliseconds,
the integration process introduces a small error in the position calculation. The accumulation of these small errors over time is what causes drift. Of course, the smaller you make the time periods
the less drift you get, but eventually you run into the limits of the CPU speed.

The Complementary Filter calculation is shown in the code below. More information about complementary filters, and how to tune them, can be found online. Since mitigating gyro drift is our primary
objective the code shows the filter being heavily weighted in that direction. */

//--------------------------------------------//
// Get current angle of the robot
//--------------------------------------------//
float IMU::currentAngle() {

  // Get raw IMU data
  readIMU();

  // Complementary filter for angle calculation
  float gRoll = getGyroRoll(gyroX, gx_offset, lastTime);
  float aRoll = getAccRoll(accY, ay_offset, accZ, az_offset);

  float angleGet = 0.98 * (angleGet + gRoll) + 0.02 * (aRoll);

  lastTime = micros(); // Reset the timer

  return angleGet;
}

/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% METHOD 2: DIGITAL MOTION PROCESSOR (DMP) (THE ONE WE ARE USING) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

/* The DMP offloads processing that would normally have to take place on the microprocessor. It maintains an internal buffer that combines data from the gyro and accelerometer
and computes orientation for you.
The DMP also takes care of the applying the offsets, so you don’t have to keep track of these in your project code. Here’s the format of the DMPs internal FiFo buffer.

                  MPU6050 FiFo BUFFER
        QUATERNION        Gyro        Accel
        w, x, y, z      x, y, z      x, y, z
          0 - 15        16 -27       28 - 39

To read the DMP’s buffer into your program you can use the following sequence of statements. The interrupt status is checked and the buffer is read into your local program variable.
Once that’s done, the orientation information can be accessed. */

// Check for DMP data ready interrupt (this should happen frequently)
if (mpuIntStatus & 0x02) {

	 // wait for correct available data length, should be a VERY short wait
	 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

   // read a packet from FIFO
   mpu.getFIFOBytes(fifoBuffer, packetSize);

   // track FIFO count here in case there is > 1 packet available
   // (this lets us immediately read more without waiting for an interrupt)
   fifoCount -= packetSize;

     // get quaternion values in easy matrix form: w x y z
   mpu.dmpGetQuaternion(&q, fifoBuffer);
}
/////////////////////////// QUATERNIONS ///////////////////////////
/* The orientation is represented by a Quaternion. Quaternions are a way to represent the orientation of an object in 3D space and can be efficiently calculated on a computer.
They also avoid a problem that occurs when you rotate through an angle of more that 90°, called Gimbal Lock. Unless you are working on a drone project you probably won’t run into the
Gimbal Lock problem so you can safely ignore it. Quaternions are somewhat difficult to understand which is why they were ignored in favor of Euler angles for over 100 years.

To understand Quaternions it’s useful to compare them to Yaw, Pitch, Roll, which is a concept that most people are more familiar with. To represent a change in orientation you first specify
the Yaw angle, which is a rotation around the z-axis. Then add the Pitch, which is a rotation around the y-axis. And finally a Roll around the x-axis. Of course, a plane may do this in a different
order, or more likely all at once, but the end result is still a change in orientation. The key here is that you only need three parameters (ψ, θ, ϕ) to represent the transformation.

Contrast this with a Quaternion that inexplicably requires four parameters. So a Quaternion is first going to use a vector and point it in the direction that you need to go. This is represented
by the red arrow in the diagram below, and is always one unit in length. Since the arrow can point anywhere is 3D space we need three parameters to define it. The direction parameters are given
as sines. Once we have a direction, we can execute a roll to get us to the final orientation. That’s the purpose of the forth parameter. It specifies in degrees (or radians) how much we need to rotate.

To ensure that the orientation data is useful for all applications the DMP stores its computations as Quaternions. Jeff Rowberg’s program gives you an easy way to convert the Quaternions to
other useful information such as Euler angles and linear acceleration. */

/////////////////////////// EULER ANGLE ///////////////////////////
/* Once we’ve retrieved data from the DMP we can use it to get Euler angles. The Quaternion values are passed into the dmpGetEuler( ) function to transform them to Euler angles.
The output is given in radians so a conversion to degrees can be done if required. The formula for converting Quaternions to Euler angles can be found online, or by examination of
Jeff Rowberg’s library. */

mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetEuler(euler, &q);

float psi = euler[0] * 180/M_PI;
float theta = euler[1] * 180/M_PI;
float phi = euler[2] * 180/M_PI;

/*Euler angles are much easier to understand than Quaternions and for some applications Euler angles are preferable. Check online to learn more about them. Euler angles are specified as a pure
change in orientation without regard to gravity. */

/////////////////////////// Yaw, Pitch, Roll ///////////////////////////
/* As you saw earlier, calculating pitch information required a bit of programming. Contrast this with getting pitch from the DMP, which can be done in four statements.
The gravity components first need to be extracted out of the precomputed Quaternions data. The gravity is then passed into a function to get the pitch. */

mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

float pitch = ypr[1] * 180/M_PI;*/

/////////////////////////// Acceleration ///////////////////////////

/* Acceleration data is also available in the DMP’s buffer. However, as the IMU is moving the force being reported by the accelerometer is not only the earth’s force but also the force
causing it to accelerate. Therefore, you need remove gravity from the calculation in order to get the linear acceleration. As mentioned previously, the gravity components can be extracted
from the Quaternions, so we use those together with the acceleration readings to compute linear acceleration. */

// display real acceleration, adjusted to remove gravity
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetAccel(&aa, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
float x = aaReal.x;
float y = aaReal.y;
float z = aaReal.z;

///////////////////////// POLL DMU (not using Pin2 conenction Arduino)/////////////////////////////
/* The MPU6050 can be used without the use of interrupts. In this mode you will be polling the IMU within the main control loop. This method is preferred if you need to send other control
actions to the robot, where you don’t want the IMU interrupts to overwhelm the CPU. This function is called in your main control loop. The DMP’s FiFo buffer is being filled much faster than
your control loop so it needs to be cleared so as we have the most recent values. We wait for the buffer to be filled and then return it to the program. */

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
