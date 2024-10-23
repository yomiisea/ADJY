#include <math.h>
#define PROVIDE_ONLY_LINEAR_MOVEMENT
// * Servidor web **
#include "ServoEasing.hpp"

#define ISNAN(X) (!((X)==(X)))

int connectionId;
int comando = 0;
int num = 0;
#define NADA 0
#define PARADO 1
#define CAMINAR 2
#define SENTARSE 3
#define DAR_PATA 4
#define PERREO 5
#define ESTIRAR 6
#define GIRARDER 7
#define GIRARIZQ 8
#define ECHARSE 9
#define RASCARSE 10
#define BAILE 11
#define CERO 12

#define DEBUG true
//JSFNKSJNDFLSKNDFLDS
const float longitudMuslo = 10;
const float longitudPierna = 10;

// para la interpolacion
//const int numFrames=17;

const int numFrames = 18; //base
const int numInterp = 4;

//------LOS NUMEROS DE FRAMES PARA MODULACION---------
const int numFrames_parado = 1;
const int numFrames_caminar = 14;
const int numFrames_sentarse = 12;
const int numFrames_darpat = 26;
const int numFrames_perreo = 18;
const int numFrames_estirarse1 = 7;
const int numFrames_estirarse2 = 5;
const int numFrames_girarder = 7;
const int numFrames_girarizq = 7;
const int numFrames_echarse = 3;
const int numFrames_rascarse = 18;
const int numFrames_baile = 9;
//---------------------------------------------------


// para la cienmatica
float returnCinematica[2];
// H1,R1,H2,R2,H3,R3,H4,R4

//----------------------------MATRICES MOVIEMNETO

const int parado[][8]  =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
};

const int caminar[][8]  =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 18.5, 0, 16,  0, 11,  0, 16}, // pata 3 arriba y adelante
  { -7, 17, 0, 16, -3, 17,  0, 16}, // pate 3 abajo y empuja
  { -7, 17, 0, 16, -3, 17,  0, 16}, // posicion 0
  { -7, 17, 3, 9, -3, 17,  0, 18}, // pata 2 arriba y adelante
  { -7, 17, 3, 16, -3, 17,  0, 16}, // pata 2 abajo y emupa
  { -7, 17, 3, 16, -3, 17,  0, 16}, // posicion 0
  { 0, 11, 3, 16, -3, 18.5,  0, 16}, // pata 1 arriba y adelante
  { -3, 17, 3, 16, -3, 17,  0, 16}, // pate 1 abajo y empuja
  { -3, 17, 3, 16, -3, 17,  0, 16}, // posicion 0
  { -3, 17, 3, 18, -3, 17,  3, 9}, // pata 4 arriba y adelante
  { -3, 17, 3, 16, -3, 17,  3, 16}, // pata 4 abajo y emupa
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0


};

const int posiciones[14][8]  =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 18.5, 0, 16,  0, 11,  0, 16}, // pata 3 arriba y adelante
  { -7, 17, 0, 16, -3, 17,  0, 16}, // pate 3 abajo y empuja
  { -7, 17, 0, 16, -3, 17,  0, 16}, // posicion 0
  { -7, 17, 3, 9, -3, 17,  0, 18}, // pata 2 arriba y adelante
  { -7, 17, 3, 16, -3, 17,  0, 16}, // pata 2 abajo y emupa
  { -7, 17, 3, 16, -3, 17,  0, 16}, // posicion 0
  { 0, 11, 3, 16, -3, 18.5,  0, 16}, // pata 1 arriba y adelante
  { -3, 17, 3, 16, -3, 17,  0, 16}, // pate 1 abajo y empuja
  { -3, 17, 3, 16, -3, 17,  0, 16}, // posicion 0
  { -3, 17, 3, 18, -3, 17,  3, 9}, // pata 4 arriba y adelante
  { -3, 17, 3, 16, -3, 17,  3, 16}, // pata 4 abajo y emupa
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0

};

const int sentarse[12][8] =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pata 3 arriba y adelante
  { -7, 13, 0, 16, -7, 12,  0, 16}, // pate 3 abajo y empuja
  { -7, 13, 1, 18, -7, 12,  1, 18}, // pate 3 abajo y empuja
  //delay
  { -5, 12, 1, 20, -5, 12,  1, 20}, // pate 3 abajo y empuja
  { -5, 12, 1, 20, -5, 12,  1, 20}, // pate 3 abajo y empuja
  { -5, 12, 1, 20, -5, 12,  1, 20}, // pate 3 abajo y empuja
  { -5, 12, 1, 20, -5, 12,  1, 20}, // pate 3 abajo y empuja
  //delay
  { -7, 13, 1, 18, -5, 13,  1, 18}, // pate 3 abajo y empuja
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pate 3 abajo y empuja
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pata 3 arriba y adelante
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0

};

const int darPat[][8] =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pata 3 arriba y adelante
  { -7, 13, 0, 16, -7, 12,  0, 16}, // pate 3 abajo y empuja
  { -7, 13, 1, 18, -7, 12,  1, 18}, // pate 3 abajo y empuja
  //delay
  { -7, 12, 1, 18, -7, 12,  1, 18}, // pate 3 abajo y empuja
  { -7, 12, 1, 18, -7, 12,  1, 18}, // pate 3 abajo y empuja

  { -7, 12, 2, 18, -7, 12,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 4, 13, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 4, 13, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 4, 13, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 8, 8, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 15, 5, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 18, 4, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 18, 4, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 15, 5, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 8, 8, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 4, 13, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 4, 13, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -8, 14, 4, 15, -8, 14,  1, 18}, // pate 3 abajo y empuja
  { -7, 12, 2, 18, -7, 12,  1, 18}, // pate 3 abajo y empuja

  { -7, 12, 1, 18, -7, 12,  1, 18}, // pate 3 abajo y empuja
  { -7, 12, 1, 18, -7, 12,  1, 18}, // pate 3 abajo y empuja
  //delay
  { -7, 13, 1, 18, -5, 13,  1, 18}, // pate 3 abajo y empuja
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pate 3 abajo y empuja
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pata 3 arriba y adelante
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0

};

const int perreo[][8] =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 17, 1, 14, -7, 17,  1, 14}, // pata 3 arriba y adelante
  { -7, 17, 1, 12, -7, 17,  1, 12}, // pate 3 abajo y empuja
  { -7, 17, 1, 10, -7, 17,  1, 10}, // pate 3 abajo y empuja
  //delay
  { -7, 12, 1, 10, -7, 12,  1, 10}, // pate 3 abajo y empuja
  { -7, 18, 1, 10, -7, 18,  1, 10}, // pate 3 abajo y empuja
  { -7, 12, 1, 10, -7, 12,  1, 10}, // pate 3 abajo y empuja
  { -7, 18, 1, 10, -7, 18,  1, 10}, // pate 3 abajo y empuja
  { -7, 12, 1, 10, -7, 12,  1, 10}, // pate 3 abajo y empuja
  { -7, 18, 1, 10, -7, 18,  1, 10}, // pate 3 abajo y empuja
  { -7, 12, 1, 10, -7, 12,  1, 10}, // pate 3 abajo y empuja
  { -7, 18, 1, 10, -7, 18,  1, 10}, // pate 3 abajo y empuja
  { -7, 12, 1, 10, -7, 12,  1, 10}, // pate 3 abajo y empuja
  { -7, 18, 1, 10, -7, 18,  1, 10}, // pate 3 abajo y empuja

  { -7, 17, 1, 10, -7, 17,  1, 10}, // pate 3 abajo y empuja
  { -7, 17, 1, 12, -7, 17,  1, 12}, // pate 3 abajo y empuja
  { -7, 17, 1, 14, -7, 17,  1, 14}, // pata 3 arriba y adelante
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0

};

const int posiciones_estirarse1[][8] =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 15, 0, 16, -7, 15,  0, 16}, // pata 3 arriba y adelante
  { -7, 15, 0, 16, -7, 15,  0, 16}, // pate 3 abajo y empuja
  { -7, 15, 1, 18, -7, 15,  1, 18}, // pate 3 abajo y empuja
  //delay
  { -7, 15, 3, 17, -7, 15,  3, 17}, // pate 3 abajo y empuja
  { -4, 15, 13, 17, -4, 15,  13, 17}, // pate 3 abajo y empuja
  { -4, 15, 15, 13, -4, 15,  15, 13}, // pate 3 abajo y empuja
  { -4, 15, 18, 13, -4, 15,  18, 13}, // pate 3 abajo y empuja
  //delay

};

const int posiciones_estirarse2[][8] =
{
  { -7, 15, 3, 17, -7, 15,  3, 17}, // pate 3 abajo y empuja
  { -7, 15, 1, 18, -7, 15,  1, 18}, // pate 3 abajo y empuja
  { -7, 15, 0, 16, -7, 15,  0, 16}, // pate 3 abajo y empuja
  { -7, 15, 0, 16, -7, 15,  0, 16}, // pata 3 arriba y adelante
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0

};

const int posiciones_girarder[][8] =
{
  { -7, 17, -4, 16, -7, 17,  -2, 16}, // posicion 0
  { -8, 17, -7, 14, 0, 13,  0, 16}, // posicion 0
  { -8, 17, -7, 14, 0, 13,  0, 16}, // posicion 0
  { -3, 17, -7, 16, -3, 17,  -2, 16}, // posicion 0
  { -3, 17, -4, 14, -3, 17,  0, 12}, // posicion 0
  { -7, 17, -4, 14, -3, 17,  0, 16}, // posicion 0
  { -7, 17, -4, 16, -7, 17,  -2, 16}, // posicion 0
};


const int posiciones_girarizq[][8] =
{
  { -7, 17, -3, 16, -6, 17,  -2, 16}, // posicion 0
  { -7, 17, 0, 9, -8, 17,  -2, 18}, // pata 2 arriba y adelante
  { -7, 17, 0, 12, -8, 10,  -2, 16}, // pata 2 abajo y emupa
  { -7, 17, -3, 16, -5, 15,  -2, 16}, // posicion 0
  { 0, 11, 0, 9, -6, 17,  -2, 16}, // pata 1 arriba y adelante
  { -3, 17, 0, 12, -6, 17,  -2, 16}, // pate 1 abajo y empuja
  { -7, 17, 2, 16, -6, 17,  -2, 16}, // posicion 0
  { -7, 17, -3, 16, -7, 17,  -2, 16}, // posicion 0
};

float posiciones_echarse[][8] =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 13, 0, 13, -7, 13,  0, 13}, // posicion 0
  { -7, 9, 0, 9, -7, 9,  0, 9}, // posicion 0
};

float posiciones_rascarse[][8] =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pata 3 arriba y adelante
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pate 3 abajo y empuja
  { -7, 13, 1, 18, -7, 13,  1, 18}, // pate 3 abajo y empuja
  { -7, 13, -4, 17, -7, 13,  -4, 17}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, -3, 8,  -4, 15}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, -1, 3,  -4, 15}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, 0, 3,  -4, 15}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, 2, 5,  -4, 15}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, 0, 3,  -4, 15}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, 2, 5,  -4, 15}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, 0, 3,  -4, 15}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, -1, 3,  -4, 15}, // pate 3 abajo y empuja
  { -11, 13, -4, 17, -9, 8,  -4, 15}, // pate 3 abajo y empuja
  { -7, 13, -4, 17, -7, 13,  -4, 17}, // pate 3 abajo y empuja
  { -7, 13, 1, 18, -7, 13,  1, 18}, // pate 3 abajo y empuja
  { -7, 13, 0, 16, -7, 13,  0, 16}, // pate 3 abajo y empuja
};

float posiciones_baile[][8] =
{
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0
  { -7, 17, -2, 14, -7, 17,  -2, 14}, // pata 3 arriba y adelante
  { -7, 17, -2, 14, -7, 17,  -2, 14}, // pate 3 abajo y empuja
  { -5, 17, -2, 14, -5, 17,  -2, 14}, // pate 3 abajo y empuja
  { -15, 12, -2, 14, -7, 17,  -2, 14}, // pate 3 abajo y empuja
  { -5, 17, -2, 14, -5, 17,  -2, 14}, // pate 3 abajo y empuja
  { -7, 17, -2, 14, -7, 17,  -2, 14}, // pate 3 abajo y empuja
  { -7, 17, -2, 14, -7, 17,  -2, 14}, // pata 3 arriba y adelante
  { -7, 17, 0, 16, -7, 17,  0, 16}, // posicion 0

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

//////////////////777
void initESP8266() {
  Serial1.begin(115200); ///////ESP Baud rate
  Serial.println("INIT");
  sendData("AT+RST\r\n", 2000, DEBUG); // reset module
  sendData("AT+CWMODE=2\r\n", 1000, DEBUG); // configure as access point
  sendData("AT+CWSAP=\"ADJY WIFI\",\"\",1,0\r\n", 2000, DEBUG); // change ssid
  sendData("AT+CIFSR\r\n", 1000, DEBUG); // get ip address
  sendData("AT+CIPMUX=1\r\n", 1000, DEBUG); // configure for multiple connections
  delay(600);
  sendData("AT+CIPSERVER=1,80\r\n", 1000, DEBUG); // turn on server on port 80
}

void keepESP8266() {
  
  Serial.println("KL");
  
  sendData("AT+CWMODE=2\r\n", 100, DEBUG); // configure as access point
  sendData("AT+CWSAP=\"ADJY WIFI\",\"\",1,0\r\n", 2000, DEBUG); // change ssid
  sendData("AT+CIFSR\r\n", 100, DEBUG); // get ip address
  sendData("AT+CIPMUX=1\r\n", 100, DEBUG); // configure for multiple connections
  delay(100);
  sendData("AT+CIPSERVER=1,80\r\n", 100, DEBUG); // turn on server on port 80
}

void espsend(String d)
{
  String cipSend = " AT+CIPSEND=";
  cipSend += connectionId;
  cipSend += ",";
  cipSend += d.length();
  cipSend += "\r\n";
  sendData(cipSend, 100, DEBUG);
  sendData(d, 100, DEBUG);
}

String sendData(String command, const int timeout, boolean debug)
{
  String response = "";
  Serial1.print(command);
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (Serial1.available())
    {
      char c = Serial1.read(); // read the next character.
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
  if (Serial1.available())
  {
    // recibe los datos del servidor web, siempre tiene que ser de la forma http://192.168.4.1?com=1&num=5
    if (Serial1.find("+IPD,"))
    {
      delay(100);
      connectionId = Serial1.read() - 48;
      if (Serial1.find("com="))
      {
        Serial.print("recibido comando de la pagina:");
        comando = Serial1.read() - 48;
        int num2 = Serial1.read() - 48;
        if (num2 >= 0 && num2 <= 9) {
          comando = comando * 10 + num2;
        }
        Serial.println(String(comando));
      }
      if (Serial1.find("num="))
      {
        Serial.print("recibido num de la pagina:");
        num = Serial1.read() - 48;
        Serial.println(String(num));
      }
      String webpage = "<h1>GUAU</h1>";
      espsend(webpage);
      String closeCommand = "AT+CIPCLOSE=";  ////////////////close the socket connection////esp command
      closeCommand += connectionId; // append connection id
      closeCommand += "\r\n";
      sendData(closeCommand, 100, DEBUG);
    }
    if (comando < 1 || comando > 99) {
      comando = 0;
    }
    if (num < 1 || num > 9) {
      num = 1;
    }
  }
}

// * FIN servidor web **

void setup()
{
  Serial.begin(115200);    // monitor
  initESP8266();  // inicializa un servidor web en el ESP8266
  delay(500);
  servoHombro1.attach(pinHombro1, calcHombro(0));
  servoRodilla1.attach(pinRodilla1, calcRodilla(0));
  servoHombro2.attach(pinHombro2, calcHombro(0));
  servoRodilla2.attach(pinRodilla2, calcRodilla(0));
  servoHombro3.attach(pinHombro3, calcHombroIz(0));
  servoRodilla3.attach(pinRodilla3, calcRodillaIz(0));
  servoHombro4.attach(pinHombro4, calcHombroIz(0));
  servoRodilla4.attach(pinRodilla4, calcRodillaIz(0));
  setSpeedForAllServos(100);
  delay(500);
}


void calcCinematica(float x, float y, float a1, float a2, float *posang) {
  float q1, q2;
  q2 = acos((x * x + y * y - a1 * a1 - a2 * a2) / (2 * a1 * a2));
  q1 = atan(y / x) - atan((a2 * sin(q2)) / (a1 + a2 * cos(q2)));
  if (ISNAN(q1)) {
    q1 = 0;
  }
  if (ISNAN(q2)) {
    q2 = 0;
  }
  q1 = q1 * 180 / 3.141592;
  q2 = q2 * 180 / 3.141592;
  //return cinematica=posang
  posang[0] = q1;
  posang[1] = q2;
  Serial.print("Q1:");
  Serial.print(q1);
  Serial.print(" Q2:");
  Serial.println(q2);
}

//PIERNA DERECHA
float calcHombro(float angulo) {
  float anguloSalida;
  anguloSalida = angulo + 90;
  if (anguloSalida > 180) {
    anguloSalida = 180;
  }
  if (anguloSalida < 0) {
    anguloSalida = 0;
  }
  // Serial.print("AngH:"); Serial.println(angulo);
  return anguloSalida;
}

float calcRodilla(float angulo) {
  float anguloSalida;
  anguloSalida = angulo;
  if (anguloSalida > 180) {
    anguloSalida = 180;
  }
  if (anguloSalida < 0) {
    anguloSalida = 0;
  }
  // Serial.print("AngR:"); Serial.println(angulo);
  return anguloSalida;
}
//PIERNA IZQUIERDA
float calcHombroIz(float angulo) {
  float anguloSalida;
  anguloSalida = -angulo + 90;
  if (anguloSalida > 180) {
    anguloSalida = 180;
  }
  if (anguloSalida < 0) {
    anguloSalida = 0;
  }
  // Serial.print("AngH:"); Serial.println(angulo);
  return anguloSalida;
}

float calcRodillaIz(float angulo) {
  float anguloSalida;
  anguloSalida = 180 - angulo;
  if (anguloSalida > 180) {
    anguloSalida = 180;
  }
  if (anguloSalida < 0) {
    anguloSalida = 0;
  }
  // Serial.print("AngR:"); Serial.println(angulo);
  return anguloSalida;
}

void setPosicionPierna1(float x, float y) {
  float anguloHombro, anguloRodilla;
  calcCinematica(x, y, longitudMuslo, longitudPierna, returnCinematica);
  anguloHombro = returnCinematica[0];
  anguloRodilla = returnCinematica[1];
  servoHombro1.setEaseTo(calcHombro(anguloHombro));
  servoRodilla1.setEaseTo(calcRodilla(anguloRodilla));
}

void setPosicionPierna2(float x, float y) {
  float anguloHombro, anguloRodilla;
  calcCinematica(x, y, longitudMuslo, longitudPierna, returnCinematica);
  anguloHombro = returnCinematica[0];
  anguloRodilla = returnCinematica[1];
  servoHombro2.setEaseTo(calcHombro(anguloHombro));
  servoRodilla2.setEaseTo(calcRodilla(anguloRodilla));
}

void setPosicionPierna3(float x, float y) {
  float anguloHombro, anguloRodilla;
  calcCinematica(x, y, longitudMuslo, longitudPierna, returnCinematica);
  anguloHombro = returnCinematica[0];
  anguloRodilla = returnCinematica[1];
  servoHombro3.setEaseTo(calcHombroIz(anguloHombro));
  servoRodilla3.setEaseTo(calcRodillaIz(anguloRodilla));
}

void setPosicionPierna4(float x, float y) {
  float anguloHombro, anguloRodilla;
  calcCinematica(x, y, longitudMuslo, longitudPierna, returnCinematica);
  anguloHombro = returnCinematica[0];
  anguloRodilla = returnCinematica[1];
  servoHombro4.setEaseTo(calcHombroIz(anguloHombro));
  servoRodilla4.setEaseTo(calcRodillaIz(anguloRodilla));
}

void printPosicion(String desc, int i, int p) {
  Serial.print(desc);
  Serial.print("Num:");
  Serial.print(i);
  Serial.print("Pos;");
  Serial.println(p);
}


//--------------------MODULARIZACION PARA ACCIONES

void parado10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_parado; p++) {
      printPosicion("PARADO", i, p);
      setPosicionPierna1(parado[p][1], parado[p][0]);
      setPosicionPierna2(parado[p][3], parado[p][2]);
      setPosicionPierna3(parado[p][5], parado[p][4]);
      setPosicionPierna4(parado[p][7], parado[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void caminar10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_caminar; p++) {
      printPosicion("CAMINAR", i, p);
      setPosicionPierna1(caminar[p][1], caminar[p][0]);
      setPosicionPierna2(caminar[p][3], caminar[p][2]);
      setPosicionPierna3(caminar[p][5], caminar[p][4]);
      setPosicionPierna4(caminar[p][7], caminar[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void sentarse10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_sentarse; p++) {
      printPosicion("SENTARSE", i, p);
      setPosicionPierna1(sentarse[p][1], sentarse[p][0]);
      setPosicionPierna2(sentarse[p][3], sentarse[p][2]);
      setPosicionPierna3(sentarse[p][5], sentarse[p][4]);
      setPosicionPierna4(sentarse[p][7], sentarse[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void dar_pat10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_darpat; p++) {
      printPosicion("DAR PATA", i, p);
      setPosicionPierna1(darPat[p][1], darPat[p][0]);
      setPosicionPierna2(darPat[p][3], darPat[p][2]);
      setPosicionPierna3(darPat[p][5], darPat[p][4]);
      setPosicionPierna4(darPat[p][7], darPat[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}
void perreo10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_perreo; p++) {
      printPosicion("PERREO", i, p);
      setPosicionPierna1(perreo[p][1], perreo[p][0]);
      setPosicionPierna2(perreo[p][3], perreo[p][2]);
      setPosicionPierna3(perreo[p][5], perreo[p][4]);
      setPosicionPierna4(perreo[p][7], perreo[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void estirar10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_estirarse1; p++) {
      printPosicion("ESTIRAR1 ", i, p);
      setPosicionPierna1(posiciones_estirarse1[p][1], posiciones_estirarse1[p][0]);
      setPosicionPierna2(posiciones_estirarse1[p][3], posiciones_estirarse1[p][2]);
      setPosicionPierna3(posiciones_estirarse1[p][5], posiciones_estirarse1[p][4]);
      setPosicionPierna4(posiciones_estirarse1[p][7], posiciones_estirarse1[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
    delay(2000);
    for (int p = 0; p < numFrames_estirarse2; p++) {
      printPosicion("ESTIRAR2 ", i, p);
      setPosicionPierna1(posiciones_estirarse2[p][1], posiciones_estirarse2[p][0]);
      setPosicionPierna2(posiciones_estirarse2[p][3], posiciones_estirarse2[p][2]);
      setPosicionPierna3(posiciones_estirarse2[p][5], posiciones_estirarse2[p][4]);
      setPosicionPierna4(posiciones_estirarse2[p][7], posiciones_estirarse2[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void girarder10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_girarder; p++) {
      printPosicion("GIRAR DER", i, p);
      setPosicionPierna1(posiciones_girarder[p][1], posiciones_girarder[p][0]);
      setPosicionPierna2(posiciones_girarder[p][3], posiciones_girarder[p][2]);
      setPosicionPierna3(posiciones_girarder[p][5], posiciones_girarder[p][4]);
      setPosicionPierna4(posiciones_girarder[p][7], posiciones_girarder[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void girarizq10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_girarizq; p++) {
      printPosicion("GIRAR IZQ", i, p);
      setPosicionPierna1(posiciones_girarizq[p][1], posiciones_girarizq[p][0]);
      setPosicionPierna2(posiciones_girarizq[p][3], posiciones_girarizq[p][2]);
      setPosicionPierna3(posiciones_girarizq[p][5], posiciones_girarizq[p][4]);
      setPosicionPierna4(posiciones_girarizq[p][7], posiciones_girarizq[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void echarse10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_echarse; p++) {
      printPosicion("ECHARSE", i, p);
      setPosicionPierna1(posiciones_echarse[p][1], posiciones_echarse[p][0]);
      setPosicionPierna2(posiciones_echarse[p][3], posiciones_echarse[p][2]);
      setPosicionPierna3(posiciones_echarse[p][5], posiciones_echarse[p][4]);
      setPosicionPierna4(posiciones_echarse[p][7], posiciones_echarse[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void rascarse10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_rascarse; p++) {
      printPosicion("RASCARSE", i, p);
      setPosicionPierna1(posiciones_rascarse[p][1], posiciones_rascarse[p][0]);
      setPosicionPierna2(posiciones_rascarse[p][3], posiciones_rascarse[p][2]);
      setPosicionPierna3(posiciones_rascarse[p][5], posiciones_rascarse[p][4]);
      setPosicionPierna4(posiciones_rascarse[p][7], posiciones_rascarse[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void baile10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  for (int i = 0; i < num; i++) {
    for (int p = 0; p < numFrames_baile; p++) {
      printPosicion("BAILE", i, p);
      setPosicionPierna1(posiciones_baile[p][1], posiciones_baile[p][0]);
      setPosicionPierna2(posiciones_baile[p][3], posiciones_baile[p][2]);
      setPosicionPierna3(posiciones_baile[p][5], posiciones_baile[p][4]);
      setPosicionPierna4(posiciones_baile[p][7], posiciones_baile[p][6]);
      synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
      do {
        delay(2);
      } while (!updateAllServos());
    }
  }
}

void cero10(int num = 1, int velocidad = 100) {
  setSpeedForAllServos(velocidad);
  servoHombro1.setEaseTo(calcHombro(0));
  servoRodilla1.setEaseTo(calcRodilla(0));
  servoHombro2.setEaseTo(calcHombro(0));
  servoRodilla2.setEaseTo(calcRodilla(0));
  servoHombro3.setEaseTo(calcHombroIz(0));
  servoRodilla3.setEaseTo(calcRodillaIz(0));
  servoHombro4.setEaseTo(calcHombroIz(0));
  servoRodilla4.setEaseTo(calcRodillaIz(0));
  synchronizeAllServosAndStartInterrupt(false); // Do not start interrupt
  do {
    delay(2);
  } while (!updateAllServos());
}

void loop()
{
  comando = 0;
  num = 1;
  webServer(); // lee el servidor web y modifica comando y num de acuerdo a lo recibido
  // ejecuta comando si fue recibido, bloquea el servidor web hasta acabar

  switch (comando) {
    default:
    case NADA:
      break;
    case PARADO:
      Serial.print("EJECUTANDO PARADO");
      parado10(num, 50);keepESP8266();
      break;
    case CAMINAR:
      Serial.print("EJECUTANDO CAMINAR");
      caminar10(num, 50);keepESP8266();
      break;
    case SENTARSE:
      Serial.print("EJECUTANDO SENTARSE");
      sentarse10(num, 50);keepESP8266();
      break;
    case DAR_PATA:
      Serial.print("EJECUTANDO DARPATA");
      dar_pat10(num, 50);keepESP8266();
      break;
    case PERREO:
      Serial.print("EJECUTANDO PERREO");
      perreo10(num, 50);keepESP8266();
      break;
    case ESTIRAR:
      Serial.print("EJECUTANDO ESTIRAR");
      estirar10(num, 50);keepESP8266();
      break;
    case GIRARDER:
      Serial.print("EJECUTANDO GIRAR DERECHA");
      girarder10(10, 50);keepESP8266();
      break;
    case GIRARIZQ:
      Serial.print("EJECUTANDO GIRAR IZQUIERDA");
      girarizq10(10, 50);keepESP8266();
      break;
    case ECHARSE:
      Serial.print("EJECUTANDO ECHARSE");
      echarse10(num, 50);keepESP8266();
      break;
    case RASCARSE:
      Serial.print("EJECUTANDO RASCARSE");
      rascarse10(num, 50);keepESP8266();
      break;
    case BAILE:
      Serial.print("EJECUTANDO BAILE");
      baile10(num, 50);keepESP8266();
      break;
    case CERO:
      Serial.print("EJECUTANDO CERO");
      cero10(1, 50);keepESP8266();
      break;

  }
}
