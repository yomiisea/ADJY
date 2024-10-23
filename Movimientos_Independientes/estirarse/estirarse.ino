  #include <math.h>

// Must specify this before the include of "ServoEasing.hpp"
//#define USE_PCA9685_SERVO_EXPANDER    // Activating this enables the use of the PCA9685 I2C expander chip/board.
//#define USE_SERVO_LIB                 // If USE_PCA9685_SERVO_EXPANDER is defined, Activating this enables force additional using of regular servo library.
#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
//#define DISABLE_COMPLEX_FUNCTIONS     // Activating this disables the SINE, CIRCULAR, BACK, ELASTIC, BOUNCE and PRECISION easings. Saves up to 1850 bytes program memory.
//#define MAX_EASING_SERVOS 3
//#define DISABLE_MICROS_AS_DEGREE_PARAMETER // Activating this disables microsecond values as (target angle) parameter. Saves 128 bytes program memory.
//#define DISABLE_MIN_AND_MAX_CONSTRAINTS    // Activating this disables constraints. Saves 4 bytes RAM per servo but strangely enough no program memory.
//#define DISABLE_PAUSE_RESUME               // Activating this disables pause and resume functions. Saves 5 bytes RAM per servo.
//#define DEBUG                              // Activating this enables generate lots of lovely debug output for this library.
//#define PRINT_FOR_SERIAL_PLOTTER           // Activating this enables generate the Arduino plotter output from ServoEasing.hpp.
#include "ServoEasing.hpp"

#define ISNAN(X) (!((X)==(X)))

const float longitudMuslo=10;
const float longitudPierna=10;

// para la interpolacion
//const int numFrames=14;
const int numFrames_sentarse1=7;
const int numFrames_sentarse2=5;
const int numFrames1=9;
const int numInterp=4;

// para la cienmatica
float returnCinematica[2];
// H1,R1,H2,R2,H3,R3,H4,R4

float posiciones3[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,13, 0,16, -7,12,  0,16}, // pate 3 abajo y empuja
  {-7,13, 1,18, -7,12,  1,18}, // pate 3 abajo y empuja
  //delay
  
  {-7,10, -4,17, -7,10,  -4,17}, // pate 3 abajo y empuja

  {-7,13, 1,18, -5,13,  1,18}, // pate 3 abajo y empuja
  {-7,13, 0,16, -7,13,  0,16}, // pate 3 abajo y empuja
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  

};


 float posiciones_sentarse1[][8]= 
{
  {-3,17, 0,16, -7,17,  0,16}, // posicion 0
  {-3,15, 0,16, -7,15,  0,16}, // pata 3 arriba y adelante 
  {-3,15, 0,16, -7,15,  0,16}, // pate 3 abajo y empuja
  {-3,15, 1,18, -7,15,  1,18}, // pate 3 abajo y empuja
  //delay
  {-7,15, 3,17, -7,15,  3,17}, // pate 3 abajo y empuja
  {-4,15, 13,12, -4,15,  13,12}, // pate 3 abajo y empuja
  {-4,15, 17,10, -4,15,  17,10}, // pate 3 abajo y empuja
  {-7,15, 17,10, -2,15,  17,10}, // pate 3 abajo y empuja
  //delay
  
}; 

float posiciones_sentarse1real[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,15, 0,16, -7,15,  0,16}, // pata 3 arriba y adelante 
  {-7,15, 0,16, -7,15,  0,16}, // pate 3 abajo y empuja
  {-7,15, 1,18, -7,15,  1,18}, // pate 3 abajo y empuja
  //delay
  {-7,15, 3,17, -7,15,  3,17}, // pate 3 abajo y empuja
  {-4,15, 13,17, -4,15,  13,17}, // pate 3 abajo y empuja
  {-4,15, 15,13, -4,15,  15,13}, // pate 3 abajo y empuja
  {-4,15, 18,10, -4,15,  18,10}, // pate 3 abajo y empuja
  //delay
  
}; 


 float posiciones_sentarse2[][8]= 
{
  {-7,15, 3,17, -7,15,  3,17}, // pate 3 abajo y empuja
  {-7,15, 1,18, -7,15,  1,18}, // pate 3 abajo y empuja
  {-7,15, 0,16, -7,15,  0,16}, // pate 3 abajo y empuja
  {-7,15, 0,16, -7,15,  0,16}, // pata 3 arriba y adelante 
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  
}; 
 
const int pinHombro1 = 2;
const int pinRodilla1 = 3;
const int pinHombro2 = 4;
const int pinRodilla2 = 5;
const int pinHombro3 = 6;
const int pinRodilla3 = 7;
const int pinHombro4 = 8;
const int pinRodilla4 = 9;
 
ServoEasing servoHombro1;
ServoEasing servoRodilla1;
ServoEasing servoHombro2;
ServoEasing servoRodilla2;
ServoEasing servoHombro3;
ServoEasing servoRodilla3;
ServoEasing servoHombro4;
ServoEasing servoRodilla4;

int posh1;

void setup (){
  Serial.begin(9600);
  delay(500);
  servoHombro1.attach(pinHombro1,calcHombro(0));
  servoRodilla1.attach(pinRodilla1,calcRodilla(0));
  servoHombro2.attach(pinHombro2,calcHombro(0));
  servoRodilla2.attach(pinRodilla2,calcRodilla(0));
  servoHombro3.attach(pinHombro3,calcHombroIz(0));
  servoRodilla3.attach(pinRodilla3,calcRodillaIz(0));
  servoHombro4.attach(pinHombro4,calcHombroIz(0));
  servoRodilla4.attach(pinRodilla4,calcRodillaIz(0));
  setSpeedForAllServos(50);
  delay(500);
}

void calcCinematica(float x,float y,float a1, float a2, float *posang){
  float q1,q2;
  q2=acos((x*x+y*y-a1*a1-a2*a2)/(2*a1*a2));
  q1=atan(y/x)-atan((a2*sin(q2))/(a1+a2*cos(q2)));
  if(ISNAN(q1)){
    q1=0;
  }
  if(ISNAN(q2)){
    q2=0;
  }
  q1=q1*180/3.141592;
  q2=q2*180/3.141592;
  //return cinematica=posang
  posang[0]=q1;
  posang[1]=q2;
  Serial.print("Q1:");
  Serial.print(q1);
  Serial.print(" Q2:");
  Serial.println(q2);
}

//PIERNA DERECHA
float calcHombro(float angulo){
  float anguloSalida;
  anguloSalida=angulo+90;
  if(anguloSalida>180){
    anguloSalida=180;
  }
  if(anguloSalida<0){
    anguloSalida=0;
  }
  // Serial.print("AngH:"); Serial.println(angulo);
  return anguloSalida;
}

float calcRodilla(float angulo){
  float anguloSalida;
  anguloSalida=angulo;
  if(anguloSalida>180){
    anguloSalida=180;
  }
  if(anguloSalida<0){
    anguloSalida=0;
  }
  // Serial.print("AngR:"); Serial.println(angulo);
  return anguloSalida;
}
//PIERNA IZQUIERDA
float calcHombroIz(float angulo){
  float anguloSalida;
  anguloSalida=-angulo+90;
  if(anguloSalida>180){
    anguloSalida=180;
  }
  if(anguloSalida<0){
    anguloSalida=0;
  }
  // Serial.print("AngH:"); Serial.println(angulo);
  return anguloSalida;
}

float calcRodillaIz(float angulo){
  float anguloSalida;
  anguloSalida=180-angulo;
  if(anguloSalida>180){
    anguloSalida=180;
  }
  if(anguloSalida<0){
    anguloSalida=0;
  }
  // Serial.print("AngR:"); Serial.println(angulo);
  return anguloSalida;
}

void setPosicionPierna1(float x, float y){
  float anguloHombro,anguloRodilla;
  calcCinematica(x,y,longitudMuslo,longitudPierna,returnCinematica);
  anguloHombro=returnCinematica[0];
  anguloRodilla=returnCinematica[1];
  servoHombro1.setEaseTo(calcHombro(anguloHombro));
  servoRodilla1.setEaseTo(calcRodilla(anguloRodilla));
}

void setPosicionPierna2(float x, float y){
  float anguloHombro,anguloRodilla;
  calcCinematica(x,y,longitudMuslo,longitudPierna,returnCinematica);
  anguloHombro=returnCinematica[0];
  anguloRodilla=returnCinematica[1];
  servoHombro2.setEaseTo(calcHombro(anguloHombro));
  servoRodilla2.setEaseTo(calcRodilla(anguloRodilla));
}

void setPosicionPierna3(float x, float y){
  float anguloHombro,anguloRodilla;
  calcCinematica(x,y,longitudMuslo,longitudPierna,returnCinematica);
  anguloHombro=returnCinematica[0];
  anguloRodilla=returnCinematica[1];
  servoHombro3.setEaseTo(calcHombroIz(anguloHombro));
  servoRodilla3.setEaseTo(calcRodillaIz(anguloRodilla));
}

void setPosicionPierna4(float x, float y){
  float anguloHombro,anguloRodilla;
  calcCinematica(x,y,longitudMuslo,longitudPierna,returnCinematica);
  anguloHombro=returnCinematica[0];
  anguloRodilla=returnCinematica[1];
  servoHombro4.setEaseTo(calcHombroIz(anguloHombro));
  servoRodilla4.setEaseTo(calcRodillaIz(anguloRodilla));
}

void printPosicion(int p){
  Serial.print("Pos ");  
  Serial.print(p);  
  Serial.print(":");  
  for(int i = 0; i < 8; i++){
    Serial.print(posiciones_sentarse1[p][i]);  
    Serial.print(",");  
  }
  Serial.println("");
}

void loop(){
  
  for(int p = 0; p < numFrames_sentarse1; p++){
    printPosicion(p);
    setPosicionPierna1(posiciones_sentarse1[p][1],posiciones_sentarse1[p][0]);
    setPosicionPierna2(posiciones_sentarse1[p][3],posiciones_sentarse1[p][2]);
    setPosicionPierna3(posiciones_sentarse1[p][5],posiciones_sentarse1[p][4]);
    setPosicionPierna4(posiciones_sentarse1[p][7],posiciones_sentarse1[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    do {
      delay(2);
    } while (!updateAllServos());
  }
  delay(5000);
  for(int p = 0; p < numFrames_sentarse2; p++){
    printPosicion(p);
    setPosicionPierna1(posiciones_sentarse2[p][1],posiciones_sentarse2[p][0]);
    setPosicionPierna2(posiciones_sentarse2[p][3],posiciones_sentarse2[p][2]);
    setPosicionPierna3(posiciones_sentarse2[p][5],posiciones_sentarse2[p][4]);
    setPosicionPierna4(posiciones_sentarse2[p][7],posiciones_sentarse2[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    do {
      delay(2);
    } while (!updateAllServos());
  }
  delay(5000);

  /*
  for(int p = 0; p < numFrames; p++){
    printPosicion(p);
    setPosicionPierna1(posiciones3[p][1],posiciones3[p][0]);
    setPosicionPierna2(posiciones3[p][3],posiciones3[p][2]);
    setPosicionPierna3(posiciones3[p][5],posiciones3[p][4]);
    setPosicionPierna4(posiciones3[p][7],posiciones3[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    if(p==4)
    {
      delay(5000);
    }
    if(p==0)
    {
      delay(5000);
    }
    do {
      delay(2);
    } while (!updateAllServos());
  }
  */
  
  
}
