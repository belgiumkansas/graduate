
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <phidget22.h>
#include <math.h>
#include <atomic>

#define PI 3.14159265
#define GRAV 9.8
#define GPU 0

using namespace std;

Madgwick test;

double linear_velocity[3];

double scale;

static void CCONV onAttachHandler(PhidgetHandle ph, void *ctx) {
  test.begin(250);
  cout << "Phidget attached!" << endl;
}

void CCONV onSpatialData(PhidgetSpatialHandle ch,
                         void *ctx,
                         const double acceleration[3],
                         const double angularRate[3],
                         const double magneticField[3],
                         double timestamp)
{
	//printf("accel: %f, %f, %f \n", acceleration[0]*9.8, acceleration[1]*9.8, acceleration[2]*9.8);
  test.updateIMU(angularRate[0], angularRate[1], angularRate[2],
                 acceleration[1], acceleration[0], acceleration[2]);

  double gravity[3];
  //X
  gravity[0] = GRAV *-1*sin(test.getPitchRadians())*cos(test.getRollRadians());
  //Y
  gravity[1] = GRAV *-1*sin(test.getRollRadians());
  //Z
  gravity[2] = GRAV *-1*cos(test.getPitchRadians())*cos(test.getRollRadians());

  double pure_accel[3];

  pure_accel[0] = acceleration[0]*GRAV - gravity[0];
  pure_accel[1] = acceleration[1]*GRAV + gravity[1];
  pure_accel[2] = acceleration[2]*GRAV + gravity[2];

  linear_velocity[0] += pure_accel[0]* .0004;
  linear_velocity[1] += pure_accel[1]* .0004;
  linear_velocity[2] += pure_accel[2]* .0004;

  double sum = 0;
  for(int i = 0; i < sizeof(linear_velocity); i++){
    sum += linear_velocity[i]*linear_velocity[i];
  }
  //scale = sqrt(sum);

  sum = -9.8;
  for(int i = 0; i < 3; i++){
    sum += acceleration[i];
  }

  scale = sum * .004;


  #ifdef DEBUG
   cout << "roll: " << test.getRoll() << " pitch: " << test.getPitch() << endl;
   cout << "X: " << gravity[0] << " Y: " << gravity[1] << " Z: " << gravity[2] << endl;
   cout << "input" << acceleration[0]*9.8<<" " << acceleration[1]*9.8<<" "<< acceleration[2]*9.8 << endl;
   cout << "pure " << pure_accel[0]<<" " << pure_accel[1]<<" "<< pure_accel[2] << endl;
  #endif


}
