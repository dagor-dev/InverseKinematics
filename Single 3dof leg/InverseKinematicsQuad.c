#include <stdio.h>
#include <math.h>


const int rr = 9;                                       // Reduction ratio of the actuator
const float knee = 170, tibia = 170, yOffset = 85;      // Segment length of the leg parts, yOffset is the offset between the shoulder motor and the axis of rotation

typedef struct Leg{
  int id;
  float theta;
  float phi;
  float gamma;
} Leg;

// Square number funtion
float sq(float numb){
  return numb*numb;
}

// Calculates the angle of the knee actuator as well as part of the angle of the hip actuator
// Takes a pointer to the struct that cointains the angles for the leg
void z(Leg *leg, float height){

  float thetaZ = ( (3.1416/2) - acos( (sq(knee) + sq(height) - sq(tibia)) / ( 2*knee*height ) ) ) * rr;

  float phiZ = ( acos( (sq(knee) + sq(tibia) - sq(height)) / ( 2*knee*tibia ) ) ) * rr;

  float *theta = &(*leg).theta;
  *theta -= thetaZ;

  float *phi = &(*leg).phi;
  *phi = -phiZ;
}

// Calculates part of the angle of the hip actuator as well as the "Leg length" considering the offset cause by inputing a non 0 x coordinate
float x(Leg *leg, float height, float distX){

  float extraTheta = ( atan( distX / height ) );

  float thetaX = extraTheta * rr;

  float newLegLength = ( height / (cos(extraTheta)) );

  //newLegLength = heightRestriction(abs(newLegLength));

  float *theta = &(*leg).theta;
  *theta = thetaX;

  return newLegLength;
}

// Calculates the hip angle
float y(Leg *leg, float height, float posY){

  float distY = yOffset + posY;

  float gammaP = atan( distY / height );
  if (isnan(gammaP)) gammaP = 3.1416/2;

  float hipHyp = distY / sin( gammaP );

  float lambda = asin ( yOffset / hipHyp );

  float gammaY = ( (  - lambda ) + gammaP  ) * rr;

  float newNewLegLength = yOffset/tan(lambda);

  //newNewLegLength = heightRestriction(abs(newNewLegLength));

  float *gamma = &(*leg).gamma;
  *gamma = gammaY;

  return newNewLegLength;
}

void inverseKinematics(Leg *leg, float pos_z, float pos_x, float pos_y){
  //int *id = &(*leg).id;
  //printf("ID: %d \n", (*leg).id);
  //*id = 2;

  z(leg, x(leg, y(leg, pos_z, pos_y), pos_x));

}

int main() {
  Leg legs[] = { {.id = 0},
                 {.id = 1},
                 {.id = 2},
                 {.id = 3} };

  int leg_amount = sizeof(legs)/sizeof(legs[0]);

  printf("Amount of legs: %d \n", leg_amount);

  for (int i = 0; i<leg_amount; i++){
    printf("ID: %d, Theta: %f, Phi: %f, Gamma: %f \n", legs[i].id, legs[i].theta, legs[i].phi, legs[i].gamma);
  }

  inverseKinematics(&legs[0], 10, 10, 5);
  inverseKinematics(&legs[1], 10, 20, 5);
  inverseKinematics(&legs[2], 10, 2, 15);
  inverseKinematics(&legs[3], 10, 0, 0);

  for (int i = 0; i<leg_amount; i++){
    printf("ID: %d, Theta: %f, Phi: %f, Gamma: %f \n", legs[i].id, legs[i].theta, legs[i].phi, legs[i].gamma);
  }

  return 0;
}
