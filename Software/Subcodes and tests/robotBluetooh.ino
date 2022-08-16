#include "BluetoothSerial.h"


// DEBUG
#define DEBUG 1
// declaramos pines
// motores
#define ENA 15
#define MR1 18
#define MR2 5
#define ENB 19
#define ML1 4
#define ML2 2
// ultrasonidos
#define ECHO1 35
#define ECHO2 33
#define ECHO3 26
#define ECHO4 14
#define TRIG1 32
#define TRIG2 25
#define TRIG3 27
#define TRIG4 12
#define DISTANCIA_MINIMA 15
#define TICK_ULTRASONIDO 20
#define VELOCIDAD 200
int distancia;
int duracion;
// pwm
const int freq = 5000;
const int PWMChannel = 0;
const int resolution = 8;
const int freq2 = 5000;
const int PWMChannel2 = 0;
const int resolution2 = 8;
// Sensores
int SensorDerecha;
int SensorAdelanteIzquierda;
int SensorAdelanteDerecha;
int SensorIzquierda;
int received;// lo que recibe la variable
char receivedChar;// received value will be stored as CHAR in this variable
int periodo = 1000;
unsigned long tiempo_actual = 0;

//---------------------------------------------------------------------------------------------------
//establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
//---------------------------------------------------------------------------------------------------
enum CAMINOS
{
  cruce_pasillo,
  cruce_pared,
};
// PIN_SENSORES en una matriz
int pin_sensores_ultrasonido[4][2] = {
    {ECHO1, TRIG1},
    {ECHO2, TRIG2},
    {ECHO3, TRIG3},
    {ECHO4, TRIG4},
};

void AsignacionpinesMOTORES()
{
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
}
// Asignacion de pines utilizando la matriz para no declarar cada sensor y hacer 100 lines de cod
void asignacionPinesSensores()
{
  for (int fila = 0; fila < 4; fila++)
  {
    int pin_echo = pin_sensores_ultrasonido[fila][0];
    int pin_trig = pin_sensores_ultrasonido[fila][1];
    pinMode(pin_echo, INPUT);
    pinMode(pin_trig, OUTPUT);
    digitalWrite(pin_trig, LOW);
  }
}
void AsignacionpinesPWM()
{
  ledcSetup(PWMChannel, freq, resolution);
  ledcSetup(PWMChannel2, freq2, resolution2);
  ledcAttachPin(ENA, PWMChannel);
  ledcAttachPin(ENB, PWMChannel2);
}

//---------------------------------------------------------------------------------------------------

enum MOTOR_ESTADO
{
  AVANZAR,
  RETROCEDER,
  STOP
};
void Motor(int estado, int pin1, int pin2, int pwm)
{

  ledcWrite(pwm, VELOCIDAD);

  switch (estado)
  {
  case AVANZAR:
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    break;
  case RETROCEDER:
    digitalWrite(pin1, HIGH );
    digitalWrite(pin2, LOW );
    break;
  case STOP:
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    break;
  }
}

void MotorDer(int estado)
{
  Motor(estado, MR1, MR2, PWMChannel);
}
void MotorIzq(int estado)
{
  Motor(estado, ML1, ML2, PWMChannel2);
}
// HACIA ADELANTE
void Forward()
{
  MotorDer(AVANZAR);
  MotorIzq(AVANZAR);
}
// HACIA ATRAS
void Backward()
{
  MotorDer(RETROCEDER);
  MotorIzq(RETROCEDER);
}
// GIRO SOBRE SU PROPIO EJE A LA DERECHA
void Right()
{
  MotorDer(RETROCEDER);
  MotorIzq(AVANZAR);
}
// GIRO SOBRE SU PROPIO EJE A LA IZQUIERDA
void Left()
{
  MotorDer(AVANZAR);
  MotorIzq(RETROCEDER);
}
// PARAR
void Stop()
{
  MotorDer(STOP);
  MotorIzq(STOP);
}
//-------------------------------------------------------------
int LeerUltrasonidos(int pin_trig, int pin_echo)
{
  long distancia;
  long duracion;
  // SENSOR
  digitalWrite(pin_trig, HIGH);
  delayMicroseconds(10); // Enviamos un pulso de 10us
  digitalWrite(pin_trig, LOW);
  duracion = pulseIn(pin_echo, HIGH);
  distancia = duracion / 58.2;
  return distancia;
}

enum MOVIMIENTOS
{
  ADELANTE,
  ATRAS,
  DERECHA,
  IZQUIERDA,
  PARAR
};
int movimiento = ADELANTE;
int movimiento_anterior = PARAR;
void MovimientodelRobot(int movimiento)
{
  switch (movimiento)
  {
  case ADELANTE:
  {
    Forward();
    break;
  }
  case ATRAS:
  {
    Backward();
    break;
  }
  case IZQUIERDA:
  {
    Left();
    break;
  }
  case DERECHA:
  {
    Right();
    delay(300);
    break;
  }
  case PARAR:
  {
    Stop();
    delay(600);
    break;
  }
  }
}

//-------------------------------------------------------------
void ImprimirEstadoRobot(int movimiento)
{
  String estado_robot = "";

  if (movimiento == ADELANTE)
    estado_robot = "ADELANTE";
  else if (movimiento == ATRAS)
    estado_robot = "ATRAS";
  else if (movimiento == IZQUIERDA)
    estado_robot = "IZQUIERDA";
  else if (movimiento == DERECHA)
    estado_robot = "DERECHA";
  else if (movimiento == PARAR)
    estado_robot = "PARAR";

  Serial.print("movimiento: ");
  Serial.print(estado_robot);
  Serial.print(" | ");
}
void ImprimirDatos(int sd, int sai, int sad, int si)
{
  Serial.print("sd: ");
  Serial.print(sd);
  Serial.print("-  sai: ");
  Serial.print(sai);
  Serial.print(" - sad: ");
  Serial.print(sad);
  Serial.print(" - si: ");
  Serial.println(si);
}
//-------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32_Benitez"); //Nombre del Bluetooth
  AsignacionpinesMOTORES();
  asignacionPinesSensores();
  AsignacionpinesPWM();
}
void loop()
{

  if (millis() > tiempo_actual + TICK_ULTRASONIDO)
  {
    tiempo_actual = millis();
   // SensorDerecha = LeerUltrasonidos(TRIG1, ECHO1);
   SensorAdelanteIzquierda = LeerUltrasonidos(TRIG2, ECHO2);
    //SensorAdelanteDerecha = LeerUltrasonidos(TRIG3, ECHO3);
   // SensorIzquierda = LeerUltrasonidos(TRIG4, ECHO4);
  }

  // logica-algoritmo

  receivedChar = (char)SerialBT.read();

  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {

    if (SensorAdelanteIzquierda > DISTANCIA_MINIMA) {
      Serial.print ("Received:");//print on serial monitor
      Serial.println(receivedChar);//print on serial monitor

      if (receivedChar == 'F')
      {
        Forward();
      }
      if (receivedChar == 'G')
      {
        Backward();
      }
      if (receivedChar == 'L')
      {
        Left();
      }
      if (receivedChar == 'R')
      {
        Right();
      }
      if (receivedChar == 'S')
      {
        Stop();
      }
    } else {
      movimiento=IZQUIERDA;
    }
  }

  if (movimiento != movimiento_anterior)
  {
    MovimientodelRobot(movimiento);
    movimiento_anterior = movimiento;
  }
  ImprimirEstadoRobot(movimiento);
  ImprimirDatos(SensorDerecha, SensorAdelanteIzquierda, SensorAdelanteDerecha, SensorIzquierda);
}