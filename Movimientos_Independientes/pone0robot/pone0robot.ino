#include <Servo.h> 
#include <math.h>

#define ISNAN(X) (!((X)==(X)))

const float longitudMuslo=10;
const float longitudPierna=10;

// para la interpolacion
const int numFrames=4;
const int numInterp=20;

// para la cienmatica
float returnCinematica[2];
// H1,R1,H2,R2,H3,R3,H4,R4
float posiciones[][8]= 
{
  {-4,14,03,14,-4,14,03,14},  // todas apoyadas
  {-4,12,03,14,-4,14,03,14},  // P1 arriba
  {-2,12,03,14,-4,14,03,14},  // P1 arriba adelante
  {-2,14,03,14,-4,14,03,14},  // P1 abajo adelante
  {-4,14,03,14,-4,14,03,14},  // P1 apoya +1
  {-4,14,03,14,-4,14,03,14},  // P4 arriba
  {-4,14,03,14,-4,14,03,14},  // P4 apoya +1
  {-4,14,03,14,-4,14,03,14},  // P3 arriba
  {-4,14,03,14,-4,14,03,14},  // P3 apoya +1
  {-4,14,03,14,-4,14,03,14},  // P2 arriba
  {-4,14,03,14,-4,14,03,14},  // P2 apoya +1
  {-4,14,03,14,-4,14,03,14}  // todas apoyadas
};

const int pinHombro1 = 2;
const int pinRodilla1 = 3;
const int pinHombro2 = 4;
const int pinRodilla2 = 5;
const int pinHombro3 = 6;
const int pinRodilla3 = 7;
const int pinHombro4 = 8;
const int pinRodilla4 = 9;
 
Servo servoHombro1;
Servo servoRodilla1;
Servo servoHombro2;
Servo servoRodilla2;
Servo servoHombro3;
Servo servoRodilla3;
Servo servoHombro4;
Servo servoRodilla4;

int posh1;

void setup (){
  Serial.begin(9600);
  servoHombro1.attach(pinHombro1);
  servoRodilla1.attach(pinRodilla1);
  servoHombro2.attach(pinHombro2);
  servoRodilla2.attach(pinRodilla2);
  servoHombro3.attach(pinHombro3);
  servoRodilla3.attach(pinRodilla3);
  servoHombro4.attach(pinHombro4);
  servoRodilla4.attach(pinRodilla4);
  servoHombro1.write(calcHombro(0));
  servoRodilla1.write(calcRodilla(0));
  servoHombro2.write(calcHombro(0));
  servoRodilla2.write(calcRodilla(0));
  servoHombro3.write(calcHombro(0));
  servoRodilla3.write(calcRodilla(0));
  servoHombro4.write(calcHombro(0));
  servoRodilla4.write(calcRodilla(0));
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
  servoHombro1.write(calcHombro(anguloHombro));
  servoRodilla1.write(calcRodilla(anguloRodilla));
}

void setPosicionPierna2(float x, float y){
  float anguloHombro,anguloRodilla;
  calcCinematica(x,y,longitudMuslo,longitudPierna,returnCinematica);
  anguloHombro=returnCinematica[0];
  anguloRodilla=returnCinematica[1];
  servoHombro2.write(calcHombro(anguloHombro));
  servoRodilla2.write(calcRodilla(anguloRodilla));
}

void setPosicionPierna3(float x, float y){
  float anguloHombro,anguloRodilla;
  calcCinematica(x,y,longitudMuslo,longitudPierna,returnCinematica);
  anguloHombro=returnCinematica[0];
  anguloRodilla=returnCinematica[1];
  servoHombro3.write(calcHombroIz(anguloHombro));
  servoRodilla3.write(calcRodillaIz(anguloRodilla));
}

void setPosicionPierna4(float x, float y){
  float anguloHombro,anguloRodilla;
  calcCinematica(x,y,longitudMuslo,longitudPierna,returnCinematica);
  anguloHombro=returnCinematica[0];
  anguloRodilla=returnCinematica[1];
  servoHombro4.write(calcHombroIz(anguloHombro));
  servoRodilla4.write(calcRodillaIz(anguloRodilla));
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


void loop(){
  servoHombro1.write(calcHombro(0));
  servoRodilla1.write(calcRodilla(0));
  servoHombro2.write(calcHombro(0));
  servoRodilla2.write(calcRodilla(0));
  servoHombro3.write(calcHombroIz(0));
  servoRodilla3.write(calcRodillaIz(0));
  servoHombro4.write(calcHombroIz(0));
  servoRodilla4.write(calcRodillaIz(0));
  delay(100);
}
