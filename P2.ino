#include <math.h>
#include "SensorReading&Actuators.h"
//

/*********************** Network Configuration ************************/

const int PatternCount = 1;
const int InputNodes = 9;   // giros (x,y,z), velocidades (x,y,z), aceleraciones (x,y,z)
const int HiddnNodes = 9;  // número de nodos ocultos (solo hay una capa)
const int OutputNodes = 3; // motor x, motor y, motor z
const int Targetsize = 9;
const float LearningRate = 0.3;
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;
const float Success = 0.0004;

float Input[PatternCount][InputNodes];

float Target[PatternCount][Targetsize] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0}
};

/*********************** End Network Configuration ************************/

int i, j, p, q, r;
int ReportEvery1000;
int RandomizedIndex[PatternCount];
long TrainingCycle;
float Rando;
float Error;
float Accum;

float Hidden[HiddenNodes];
float Output[OutputNodes];
float MotorIn[OutputNodes];
float * statevector_ptr[Targetsize]; //This is SensorOut
float HiddenWeights[InputNodes+1][HiddenNodes];
float OutputWeights[HiddenNodes+1][OutputNodes];
float HiddenDelta[HiddenNodes];
float OutputDelta[OutputNodes];
float ChangeHiddenWeights[InputNodes+1][HiddenNodes];
float ChangeOutputWeights[HiddenNodes+1][OutputNodes];

void initialiseNN() {

/************* Initialize HiddenWeights and ChangeHiddenWeights ****************/

  for( i = 0 ; i < HiddenNodes ; i++ ) {
    for( j = 0 ; j <= InputNodes ; j++ ) {
      ChangeHiddenWeights[j][i] = 0.0 ;
      Rando = float(random(100))/100;
      HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }

/************* Initialize OutputWeights and ChangeOutputWeights ****************/

  for( i = 0 ; i < OutputNodes ; i ++ ) {
    for( j = 0 ; j <= HiddenNodes ; j++ ) {
      ChangeOutputWeights[j][i] = 0.0 ;
      Rando = float(random(100))/100;
      OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    }
  }
  // Serial.println("Initial/Untrained Outputs: ");
}

/*********************** Hardware ************************/

Sensor sensor();
Actuator motor();

/******************************************************************
* Void Setup
******************************************************************/
void setup(){
  Serial.begin(115200);
  randomSeed(analogRead(3));
  ReportEvery1000 = 1;
  for( p = 0 ; p < PatternCount ; p++ ) {
    RandomizedIndex[p] = p ;
  }
  //readSensor();
  initialiseNN();
}

/******************************************************************
* Void Loop
******************************************************************/
void loop (){
  NeuralNetwork();
}

/******************************************************************
* NeuralNetwork()
******************************************************************/
void NeuralNetwork() {


/*********************** Begin training ************************/

  for( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {    // como un while

/********** Randomize order of training patterns *****************/

/* Mini Batch Method? */

    for( p = 0 ; p < PatternCount ; p++) {
      q = random(PatternCount);
      r = RandomizedIndex[p] ;
      RandomizedIndex[p] = RandomizedIndex[q] ;
      RandomizedIndex[q] = r ;
    }
    Error = 0.0 ;

/********** Cycle through each training pattern in the randomized order *****************/

    for( q = 0 ; q < PatternCount ; q++ ) {
      p = RandomizedIndex[q];

/********** Read sensor outputs *****************/

      statevector_ptr = &sensor.readSensor();

      Input[p][] = *statevector_ptr[];

/********** Compute hidden layer activations *****************/

      for( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = HiddenWeights[InputNodes][i] ;
        for( j = 0 ; j < InputNodes ; j++ ) {
          Accum += Input[p][j] * HiddenWeights[j][i] ;  // adds inputs (matriz con inputs de cada nodo) to weights
        }
        Hidden[i] = 1.0/(1.0 + exp(-Accum)) ;           // Sigmoid activation function
      }

/********** Compute output layer activations and calculate errors *****************/

      for( i = 0 ; i < OutputNodes ; i++ ) {
        Accum = OutputWeights[HiddenNodes][i] ;
        for( j = 0 ; j < HiddenNodes ; j++ ) {
          Accum += Hidden[j] * OutputWeights[j][i] ;
        }
        Output[i] = 1.0/(1.0 + exp(-Accum)) ;         // Sigmoid activation function

        MotorIn[i] = Output[i] * 255;
      }

      //motor.directionControl(MotorIn[]);
      //SensorOut = sensor.readSensor();

      for( i = 0 ; i < Targetsize; i++ ) {
        OutputDelta[i] = (Target[p][i] - SensorOut[i]) * Output[i] * (1.0 - Output[i]) ;  // error por derivada de función de activación
        // target es nuestro comando // SensorOut es lectura después de que se activen los motores
        Error += 0.5 * (Target[p][i] - SensorOut[i]) * (Target[p][i] - SensorOut[i]) ;
      }

/********** Backpropagate errors to hidden layer *****************/

      for( i = 0 ; i < HiddenNodes ; i++ ) {
        Accum = 0.0 ;
        for( j = 0 ; j < OutputNodes ; j++ ) {
          Accum += OutputWeights[i][j] * OutputDelta[j] ;
        }
        HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;
      }

/********** Update Inner-->Hidden Weights *****************/

      for( i = 0 ; i < HiddenNodes ; i++ ) {
        ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;

        HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;

        for( j = 0 ; j < InputNodes ; j++ ) {
          ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];

          HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;
        }
      }

/********** Update Hidden-->Output Weights *****************/

      for( i = 0 ; i < OutputNodes ; i ++ ) {
        ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;

        OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;

        for( j = 0 ; j < HiddenNodes ; j++ ) {
          ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;

          OutputWeights[j][i] += ChangeOutputWeights[j][i] ;
        }
      }

/********** If error rate is less than pre-determined threshold then end *****************/

    if( Error < Success ) break ;
    }
  }
}
