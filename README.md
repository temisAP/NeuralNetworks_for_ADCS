### NeuralNetworks_for_ADCS ###
Arduino Neural Network for Attitude Determination and Control Systems

This repository consists in one main code (P2.ino) and one library (SensorReadingsActuators) which must be included in the library directory of Arduino. Also two more libraries must be included in order to get the gyroscope (MPU-6050) outputs: I2Cdev and MPU-6050

# P2.ino

The main code is a modification over NN.ino where neural-network outputs are the reaction wheels inputs and where the error is calculated comparing a target statevector with the current statevector got from MPU-6050 output.

First neural-network parameters are modified to fit ADCS needs.
  -The network works and learn continously.
  -The network outputs are not compared directly with target but sensor outputs are compred with target.

# SensorReadingsActuators

This code contains class definition for 'Sensor' and 'Actuator' and its method members aimed to get data (readSensor()) and to move the reaction wheels.
