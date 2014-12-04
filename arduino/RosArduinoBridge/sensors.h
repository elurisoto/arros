/* Functions for various sensor types */

#ifndef SENSORS_H
#define SENSORS_H

#include <NewPing.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <QTRSensors.h>

#define IR_SENSORS 8  //Number of IR sensors used
#define TIMEOUT 2500
#define EMITTER_PIN 22
#define SONAR_NUM 6
#define MAX_DISTANCE 300

QTRSensorsRC qtrrc((unsigned char[]) {23, 24, 25, 26, 27, 28, 29, 30},
  IR_SENSORS, TIMEOUT, EMITTER_PIN);

// Calibrado para el suelo de mi casa

static int minimosOn[] = {1108, 768, 716, 664, 664, 612, 716, 824 };
static int maximosOn[] = {2500, 2500, 2252, 2020, 1976, 1976, 2480, 2500 };

unsigned int sensorValues[IR_SENSORS];

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void inicializaSeguidor(){
  qtrrc.calibratedMaximumOn = (unsigned int*)malloc(sizeof(unsigned int)*IR_SENSORS);
  qtrrc.calibratedMinimumOn = (unsigned int*)malloc(sizeof(unsigned int)*IR_SENSORS);
  // qtrrc.calibratedMinimumOn = (unsigned int[]) {516, 360, 360, 356, 312, 312, 364, 416 }; 
  // qtrrc.calibratedMaximumOn = (unsigned int[]) {2248, 1612, 1452, 1292, 1344, 1344, 1728, 2132};
  for (int i = 0; i < IR_SENSORS; i++){
    qtrrc.calibratedMaximumOn[i] = maximosOn[i];
    qtrrc.calibratedMinimumOn[i] = minimosOn[i];
  }

}

int leeSeguidor(){
  return qtrrc.readLine(sensorValues);
}

NewPing sonar[SONAR_NUM] = {
  NewPing(40,40,MAX_DISTANCE),  // Verde
  NewPing(41,41,MAX_DISTANCE),  // Azul
  NewPing(42,42,MAX_DISTANCE),  // Amarillo
  NewPing(43,43,MAX_DISTANCE),  // Rojo
  NewPing(44,44,MAX_DISTANCE),  // Negro
  NewPing(45,45,MAX_DISTANCE),  // Bicolor
};

unsigned int Ping(int indice) {
  unsigned int range;

  range = sonar[indice].ping_cm();
  return(range);
}

void leeSonars(unsigned int* v){

  for (int i = 0; i < SONAR_NUM; i++){
    v[i] = sonar[i].ping_cm();
  }

}

#endif
