#include<SoftwareSerial.h>
#include <math.h>
#define PROVIDE_ONLY_LINEAR_MOVEMENT  // Activating this disables all but LINEAR movement. Saves up to 1540 bytes program memory.
#include "ServoEasing.hpp"
#define ISNAN(X) (!((X)==(X)))


// ** Servidor web ***

SoftwareSerial esp8266(10, 11);

int connectionId;
int comando = 0;
int num = 0;
#define NADA 0
#define CAMINAR 1
#define SENTADO 2
#define HECHARSE 3
#define PATITA 4
#define ESTIRAR 5
#define GIRAR_DERECHA 6
#define GIRAR_IZQUIERDA 7
#define RASCARSE 8
#define PARARSE 9
#define PERREO 10
#define PERREOXD 11

#define DEBUG true

const float longitudMuslo=10;
const float longitudPierna=10;
const int numInterp=4;

//------LOS NUMEROS DE FRAMES PARA MODULACION---------
const int numFrames_caminar=14;
const int numFrames_sentarse=5;
const int numFrames_hecharse=3;
const int numFrames_darpat=26;
const int numFrames_estirarse=8;
const int numFrames_girardere=7;
const int numFrames_girarizqui=8;
const int numFrames_perreo=18;


//---------------------------------------------------
//PINES DE La PATAS_________________

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

//______________________________________
// para la cienmatica
float returnCinematica[2];
// H1,R1,H2,R2,H3,R3,H4,R4

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

const int sentarse[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,13, 0,16, -7,13,  0,16}, // pata 3 arriba y adelante 
  {-7,13, 0,16, -7,12,  0,16}, // pate 3 abajo y empuja
  {-7,13, 1,18, -7,12,  1,18}, // pate 3 abajo y empuja
  //delay
  {-7,10, -4,17, -7,10,  -4,17}, // pate 3 abajo y empuja
  //delay
  
}; 
const int hecharse[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
    {-7,13, 0,13, -7,13,  0,13}, // posicion 0
    {-7,9, 2,9, -7,9,  2,9}, // posicion 0
  
}; 

const int darPat[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
    {-7,13, 0,13, -7,13,  0,13}, // posicion 0
    {-7,9, 2,9, -7,9,  2,9}, // posicion 0
  
}; 

const int estirarse[][8]= 
{
  {-7,17, 0,16, -7,17,  0,16}, // posicion 0
  {-7,15, 0,16, -7,15,  0,16}, // pata 3 arriba y adelante 
  {-7,15, 0,16, -7,15,  0,16}, // pate 3 abajo y empuja
  {-7,15, 1,18, -7,15,  1,18}, // pate 3 abajo y empuja
  //delay
  {-7,15, 3,17, -7,15,  3,17}, // pate 3 abajo y empuja
  {-4,15, 13,17, -4,15,  13,17}, // pate 3 abajo y empuja
  {-4,15, 15,13, -4,15,  15,13}, // pate 3 abajo y empuja
  {-4,15, 18,13, -4,15,  18,13}, // pate 3 abajo y empuja
  //delay
  
}; 

const int girarDere[][8]= 
{
  {-7,17, -4,16, -7,17,  -2,16}, // posicion 0

  {-8,17, -7,14, 0,13,  0,16}, // posicion 0
  
  {-8,17, -7,14, 0,13,  0,16}, // posicion 0

  {-3,17, -7,16, -3,17,  -2,16}, // posicion 0

  {-3,17, -4,14, -3,17,  0,12}, // posicion 0

  {-7,17, -4,14, -3,17,  0,16}, // posicion 0
  
  {-7,17, -4,16, -7,17,  -2,16}, // posicion 0
  
}; 

const int girarIzq[][8]= 
{
  {-7,17, -3,16, -6,17,  -2,16}, // posicion 0
  {-7,17, 0, 9, -8,17,  -2,18}, // pata 2 arriba y adelante
  {-7,17, 0,12, -8,10,  -2,16}, // pata 2 abajo y emupa
  {-7,17, -3,16, -5,15,  -2,16}, // posicion 0
  { 0,11, 0,9, -6, 17,  -2,16}, // pata 1 arriba y adelante 
  {-3,17, 0,12, -6,17,  -2,16}, // pate 1 abajo y empuja
  {-7,17, 2,16, -6,17,  -2,16}, // posicion 0
  {-7,17, -3,16, -7,17,  -2,16}, // posicion 0
  
}; 



void initESP8266() {
  esp8266.begin(9600); ///////ESP Baud rate
  Serial.println("INIT");
  sendData("AT+RST\r\n", 2000, DEBUG); // reset module
  sendData("AT+CWMODE=2\r\n", 1000, DEBUG); // configure as access point
  sendData("AT+CWSAP=\"ADJY WIFI\",\"\",1,0\r\n", 2000, DEBUG); // change ssid
  sendData("AT+CIFSR\r\n", 1000, DEBUG); // get ip address
  sendData("AT+CIPMUX=1\r\n", 1000, DEBUG); // configure for multiple connections
  delay(1000);
  sendData("AT+CIPSERVER=1,80\r\n", 1000, DEBUG); // turn on server on port 80
}

void espsend(String d)
{
  String cipSend = " AT+CIPSEND=";
  cipSend += connectionId;
  cipSend += ",";
  cipSend += d.length();
  cipSend += "\r\n";
  sendData(cipSend, 1000, DEBUG);
  sendData(d, 1000, DEBUG);
}

String sendData(String command, const int timeout, boolean debug)
{
  String response = "";
  esp8266.print(command);
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (esp8266.available())
    {
      char c = esp8266.read(); // read the next character.
      response += c;
    }
  }
  if (debug)
  {
    Serial.print(response); //displays the esp response messages in arduino Serial monitor
  }
  return response;
}

void webServer() {
  if (esp8266.available())
  {
    // recibe los datos del servidor web, siempre tiene que ser de la forma http://192.168.4.1?com=1&num=5
    if (esp8266.find("+IPD,"))
    {
      delay(300);
      connectionId = esp8266.read() - 48;
      if (esp8266.find("com="))
      {
        Serial.print("recibido comando de la pagina:");
        comando = esp8266.read() - 48;
        Serial.println(String(comando));
      }
      if (esp8266.find("num="))
      {
        Serial.print("recibido num de la pagina:");
        num = esp8266.read() - 48;
        Serial.println(String(num));
      }
      String webpage = "<h1>GUAU</h1>";
      espsend(webpage);
      String closeCommand = "AT+CIPCLOSE=";  ////////////////close the socket connection////esp command
      closeCommand += connectionId; // append connection id
      closeCommand += "\r\n";
      sendData(closeCommand, 3000, DEBUG);
    }
    if (comando < 1 || comando > 9) {
      comando = 0;
    }
    if (num < 1 || num > 9) {
      num = 0;
    }
  }
}

// ** FIN servidor web ***

void setup()
{
  Serial.begin(9600);    ///////For Serial monitor
  initESP8266();  // inicializa un servidor web en el ESP8266
  //COSAS DE ADJY 
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
  //¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿
}
//FUNCIONES PARA MOVER LAS PATAS

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
    //Serial.print(posiciones[p][i]);  
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
//¿_____________________________________________-
void loop()
{
  comando = 0;
  num = 0;
  webServer(); // lee el servidor web y modifica comando y num de acuerdo a lo recibido
  // ejecuta comando si fue recibido, bloquea el servidor web hasta acabar
  switch (comando) {
    default:
    case NADA:
      break;
    case CAMINAR:
      Serial.print("EJECUTANDO PASO");
      break;
    case SENTADO:
      Serial.print("EJECUTANDO SENTADO");
      break;
    case HECHARSE:
      Serial.print("EJECUTANDO PASO");
      break;
    case PATITA:
      Serial.print("EJECUTANDO SENTADO");
      break;
    case ESTIRAR:
      Serial.print("EJECUTANDO PASO");
      break;
    case GIRAR_DERECHA:
      Serial.print("EJECUTANDO SENTADO");
      break;
    case GIRAR_IZQUIERDA:
      Serial.print("EJECUTANDO PASO");
      break;
    case RASCARSE:
      Serial.print("EJECUTANDO SENTADO");
      break;
     case PARARSE:
      Serial.print("EJECUTANDO SENTADO");
      break;
    case PERREO:
      Serial.print("EJECUTANDO SENTADO");
      break;
    case PERREOXD:
      Serial.print("EJECUTANDO SENTADO");
      break;
  }
}
