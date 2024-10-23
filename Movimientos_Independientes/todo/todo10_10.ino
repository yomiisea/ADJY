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
//const int numFrames=17;

const int numFrames=18;//base
const int numInterp=4;

//------LOS NUMEROS DE FRAMES PARA MODULACION---------
const int numFrames_caminar=14;
const int numFrames_sentarse=12;
const int numFrames_darpat=26;
const int numFrames_perreo=18;
const int numFrames_hecharse=16;
const int numFrames_rascarse=21;
//---------------------------------------------------


// para la cienmatica
float returnCinematica[2];
// H1,R1,H2,R2,H3,R3,H4,R4

//----------------------------MATRICES MOVIEMNETO
const int caminar[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,18.5, 0,16,  0,11,  0,16}, // pata 3 arriba y adelante 
  {-7,17, 0,16, -3,17,  0,16}, // pate 3 abajo y empuja
  {-7,17, 0,16, -3,17,  0,16}, // posicion 0
  {-7,17, 3, 9, -3,17,  0,18}, // pata 2 arriba y adelante
  {-7,17, 3,16, -3,17,  0,16}, // pata 2 abajo y emupa
  {-7,17, 3,16, -3,17,  0,16}, // posicion 0
  { 0,11, 3,16, -3,18.5,  0,16}, // pata 1 arriba y adelante 
    {-3,17, 3,16, -3,17,  0,16}, // pate 1 abajo y empuja
  {-3,17, 3,16, -3,17,  0,16}, // posicion 0
  {-3,17, 3,18, -3,17,  3, 9}, // pata 4 arriba y adelante
  {-3,17, 3,16, -3,17,  3,16}, // pata 4 abajo y emupa
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  

};

const int posiciones[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,18.5, 0,16,  0,11,  0,16}, // pata 3 arriba y adelante 
  {-7,17, 0,16, -3,17,  0,16}, // pate 3 abajo y empuja
  {-7,17, 0,16, -3,17,  0,16}, // posicion 0
  {-7,17, 3, 9, -3,17,  0,18}, // pata 2 arriba y adelante
  {-7,17, 3,16, -3,17,  0,16}, // pata 2 abajo y emupa
  {-7,17, 3,16, -3,17,  0,16}, // posicion 0
  { 0,11, 3,16, -3,18.5,  0,16}, // pata 1 arriba y adelante 
  {-3,17, 3,16, -3,17,  0,16}, // pate 1 abajo y empuja
  {-3,17, 3,16, -3,17,  0,16}, // posicion 0
  {-3,17, 3,18, -3,17,  3, 9}, // pata 4 arriba y adelante
  {-3,17, 3,16, -3,17,  3,16}, // pata 4 abajo y emupa
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
 
}; 

const int sentarse[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,13, 0,16, -7,12,  0,16}, // pate 3 abajo y empuja
  {-7,13, 1,18, -7,12,  1,18}, // pate 3 abajo y empuja
  //delay
  {-5,12, 1,20, -5,12,  1,20}, // pate 3 abajo y empuja
  {-5,12, 1,20, -5,12,  1,20}, // pate 3 abajo y empuja
  {-5,12, 1,20, -5,12,  1,20}, // pate 3 abajo y empuja
  {-5,12, 1,20, -5,12,  1,20}, // pate 3 abajo y empuja
  {-5,12, 1,20, -5,12,  1,20}, // pate 3 abajo y empuja
  {-5,12, 1,20, -5,12,  1,20}, // pate 3 abajo y empuja
  {-5,12, 1,20, -5,12,  1,20}, // pate 3 abajo y empuja
  {-5,12, 1,20, -5,12,  1,20}, // pate 3 abajo y empuja
  //delay
  {-7,13, 1,18, -5,13,  1,18}, // pate 3 abajo y empuja
  {-7,13, 0,16, -7,13,  0,16}, // pate 3 abajo y empuja
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  
}; 
const int hecharse[][8]= 
{
   {-7,17, 0,16, -7,17,  0,16}, // posicion 0
    {-7,13, 0,13, -7,13,  0,13}, // posicion 0
    {-7,9, 2,9, -7,9,  2,9}, // posicion 0
    {-7,9, 2,9, -7,9,  2,9}, // posicion 0

{-7,9, 2,9, -7,9,  2,9}, // posicion 0

{-7,9, 2,9, -7,9,  2,9}, // posicion 0
{-7,9, 2,9, -7,9,  2,9}, // posicion 0
{-7,9, 2,9, -7,9,  2,9}, // posicion 0
{-7,9, 2,9, -7,9,  2,9}, // posicion 0
{-7,9, 2,9, -7,9,  2,9}, // posicion 0
{-7,9, 2,9, -7,9,  2,9}, // posicion 0
{-7,9, 2,9, -7,9,  2,9}, // posicion 0
{-7,9, 2,9, -7,9,  2,9}, // posicion 0


    {-7,9, 0,9, -7,9,  0,9}, // posicion 0
    {-7,13, 0,13, -7,13,  0,13}, // posicion 0
    {-7,17, 0,16, -7,17,  0,16}, // posicion 0
    
}; 

const int darPat[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,13, 0,16, -7,12,  0,16}, // pate 3 abajo y empuja
  {-7,13, 1,18, -7,12,  1,18}, // pate 3 abajo y empuja
  //delay
  {-7,12, 1,18, -7,12,  1,18}, // pate 3 abajo y empuja
  {-7,12, 1,18, -7,12,  1,18}, // pate 3 abajo y empuja
  
  {-7,12, 2,18, -7,12,  1,18}, // pate 3 abajo y empuja
  {-8,14, 4,13, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 4,13, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 4,13, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 8,8, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 15,5, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 18,4, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 18,4, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 15,5, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 8,8, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 4,13, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 4,13, -8,14,  1,18}, // pate 3 abajo y empuja
  {-8,14, 4,15, -8,14,  1,18}, // pate 3 abajo y empuja
  {-7,12, 2,18, -7,12,  1,18}, // pate 3 abajo y empuja
  
  {-7,12, 1,18, -7,12,  1,18}, // pate 3 abajo y empuja
  {-7,12, 1,18, -7,12,  1,18}, // pate 3 abajo y empuja
  //delay
  {-7,13, 1,18, -5,13,  1,18}, // pate 3 abajo y empuja
  {-7,13, 0,16, -7,13,  0,16}, // pate 3 abajo y empuja
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
 
}; 
const int rascarse[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,13, 0,16, -7,13,  0,16}, // pate 3 abajo y empuja
  {-7,13, 1,18, -7,13,  1,18}, // pate 3 abajo y empuja
  //delay
  {-7,13, -4,17, -7,13,  -4,17}, // pate 3 abajo y empuja
  {-7,13, -4,17, -7,13,  -4,17}, // pate 3 abajo y empuja

  {-11,13, -4,17, -3,8,  -4,15}, // pate 3 abajo y empuja
  {-11,13, -4,17, -1,3,  -4,15}, // pate 3 abajo y empuja
  {-11,13, -4,17, 0,3,  -4,15}, // pate 3 abajo y empuja
  {-11,13, -4,17, 2,5,  -4,15}, // pate 3 abajo y empuja
  {-11,13, -4,17, 0,3,  -4,15}, // pate 3 abajo y empuja
  {-11,13, -4,17, 2,5,  -4,15}, // pate 3 abajo y empuja
  {-11,13, -4,17, 0,3,  -4,15}, // pate 3 abajo y empuja
  {-11,13, -4,17, -1,3,  -4,15}, // pate 3 abajo y empuja
  {-11,13, -4,17, -9,8,  -4,15}, // pate 3 abajo y empuja
  
  {-7,13, -4,17, -7,13,  -4,17}, // pate 3 abajo y empuja
  {-7,13, -4,17, -7,13,  -4,17}, // pate 3 abajo y empuja
  //delay
  {-7,13, 1,18, -7,13,  1,18}, // pate 3 abajo y empuja
  {-7,13, 0,16, -7,13,  0,16}, // pate 3 abajo y empuja
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0

}; 
const int perreo[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,17, 1,14, -7,17,  1,14}, // pata 3 arriba y adelante 
  {-7,17, 1,12, -7,17,  1,12}, // pate 3 abajo y empuja
  {-7,17, 1,10, -7,17,  1,10}, // pate 3 abajo y empuja
  //delay
  {-7,12, 1,10, -7,12,  1,10}, // pate 3 abajo y empuja
  {-7,18, 1,10, -7,18,  1,10}, // pate 3 abajo y empuja
  {-7,12, 1,10, -7,12,  1,10}, // pate 3 abajo y empuja
  {-7,18, 1,10, -7,18,  1,10}, // pate 3 abajo y empuja
  {-7,12, 1,10, -7,12,  1,10}, // pate 3 abajo y empuja
  {-7,18, 1,10, -7,18,  1,10}, // pate 3 abajo y empuja
  {-7,12, 1,10, -7,12,  1,10}, // pate 3 abajo y empuja
  {-7,18, 1,10, -7,18,  1,10}, // pate 3 abajo y empuja
  {-7,12, 1,10, -7,12,  1,10}, // pate 3 abajo y empuja
  {-7,18, 1,10, -7,18,  1,10}, // pate 3 abajo y empuja
  
  {-7,17, 1,10, -7,17,  1,10}, // pate 3 abajo y empuja
  {-7,17, 1,12, -7,17,  1,12}, // pate 3 abajo y empuja
  {-7,17, 1,14, -7,17,  1,14}, // pata 3 arriba y adelante
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
 
}; 

//----------------------------------------------------------------

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
  setSpeedForAllServos(100);
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
    Serial.print(posiciones[p][i]);  
    Serial.print(",");  
  }
  Serial.println("");
}
//--------------------MODULACION PARA ACCIONES
void caminar10(){
  for(int p = 0; p < numFrames_caminar; p++){
    printPosicion(p);
    setPosicionPierna1(caminar[p][1],caminar[p][0]);
    setPosicionPierna2(caminar[p][3],caminar[p][2]);
    setPosicionPierna3(caminar[p][5],caminar[p][4]);
    setPosicionPierna4(caminar[p][7],caminar[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    do {
      delay(2);
    } while (!updateAllServos());
  }
}

void sentarse10(){
  for(int p = 0; p < numFrames_sentarse; p++){
    printPosicion(p);
    setPosicionPierna1(sentarse[p][1],sentarse[p][0]);
    setPosicionPierna2(sentarse[p][3],sentarse[p][2]);
    setPosicionPierna3(sentarse[p][5],sentarse[p][4]);
    setPosicionPierna4(sentarse[p][7],sentarse[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    do {
      delay(2);
    } while (!updateAllServos());
  }
}

void dar_pat10(){
  for(int p = 0; p < numFrames_darpat; p++){
    printPosicion(p);
    setPosicionPierna1(darPat[p][1],darPat[p][0]);
    setPosicionPierna2(darPat[p][3],darPat[p][2]);
    setPosicionPierna3(darPat[p][5],darPat[p][4]);
    setPosicionPierna4(darPat[p][7],darPat[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    do {
      delay(2);
    } while (!updateAllServos());
  }
}
void hecharse10(){
  for(int p = 0; p < numFrames_hecharse; p++){
    printPosicion(p);
    setPosicionPierna1(hecharse[p][1],hecharse[p][0]);
    setPosicionPierna2(hecharse[p][3],hecharse[p][2]);
    setPosicionPierna3(hecharse[p][5],hecharse[p][4]);
    setPosicionPierna4(hecharse[p][7],hecharse[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    do {
      delay(2);
    } while (!updateAllServos());
  }
}
void rascarse10(){
  for(int p = 0; p < numFrames; p++){
    printPosicion(p);
    setPosicionPierna1(rascarse[p][1],rascarse[p][0]);
    setPosicionPierna2(rascarse[p][3],rascarse[p][2]);
    setPosicionPierna3(rascarse[p][5],rascarse[p][4]);
    setPosicionPierna4(rascarse[p][7],rascarse[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    do {
      delay(2);
    } while (!updateAllServos());
  }
}
void perreo10(){
  for(int p = 0; p < numFrames_perreo; p++){
    printPosicion(p);
    setPosicionPierna1(perreo[p][1],perreo[p][0]);
    setPosicionPierna2(perreo[p][3],perreo[p][2]);
    setPosicionPierna3(perreo[p][5],perreo[p][4]);
    setPosicionPierna4(perreo[p][7],perreo[p][6]);
    synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
    do {
      delay(2);
    } while (!updateAllServos());
  }
}
//---------------------------------------------------------------------------
void loop(){
 
  for(int n= 0; n < 1; n++)//CUANTAS VECES REPITE LA MATRIZ DE CAMINAR
  {
      caminar10();
   }
   delay(5000);
   for(int q= 0; q < 2; q++)//CUANTAS VECES REPITE LA MATRIZ DE SENTARSE
  {
      sentarse10();
      delay(3000);
   }
    delay(5000);
   for(int k= 0; k < 2; k++)//CUANTAS VECES REPITE LA MATRIZ DE DAR PATA
  {
      dar_pat10();
      delay(3000);
   }
    
   for(int l= 0; l < 2; l++)//CUANTAS VECES REPITE LA MATRIZ DE PERREO
  {
      perreo10();
   }
    for(int a= 0; a < 2; a++)//CUANTAS VECES REPITE LA MATRIZ DE hecharse
  {
      hecharse10();
      delay(3000);
   }
    delay(5000);
     for(int l= 0; l < 2; l++)//CUANTAS VECES REPITE LA MATRIZ DE hecharse
  {
      rascarse10();
   }
    delay(5000);
}
