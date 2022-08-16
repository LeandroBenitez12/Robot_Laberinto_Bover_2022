#include "BluetoothSerial.h"
// DEBUG
#define DEBUG_DE_CASOS 1
#define DEBUG_DE_SENSORES 1
#define DEBUG_DE_BUZZER 1
// declaramos pines
// motores
#define ENA 15
#define MR1 18
#define MR2 5
#define ENB 19
#define ML1 4
#define ML2 2
/
#define DISTANCIA_MINIMA_ADE 15
#define TICK_STOP 1000
#define TICK_ULTRASONIDO 10
#define VELOCIDAD 140
#define BUZZER 23
#define BUTTON 13
// button
bool button_start;

int periodo = 1000;
#define PERIODO 100
unsigned long tiempo_actual = 0;
unsigned long tiempo_actual_stop = 0;
unsigned long tiempo_actual_button = 0;
//------------------------------------------------------------------------------------------
void AsignacionpinesMOTORES()
{
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(BUTTON, INPUT);
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
      {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        break;
      }
    case RETROCEDER:
      {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        break;
      }
    case STOP:
      {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        break;
      }
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
void Buzzer()
{
  digitalWrite(BUZZER, HIGH);
}
//-------------------------------------------------------------
int LeerButton(int button)
{
  bool estado_button;
  estado_button = digitalRead(button);
  return estado_button;
}
//-------------------------------------------------------------
void ImprimirEstadoRobot(int rival)
{
  String estado_robot = "";
  if (rival == INICIAL)
    estado_robot = "STANDBY";
  else if (rival == )
    estado_robot = "PASILLO";
  else if (rival == FRONTAL)
    estado_robot = "FRONTAL";
  else if (rival == FRONTAL_DERECHO)
    estado_robot = "HAY ALGO A LA DERECHA";
  else if (rival == IZQUIERDO_FRONTAL)
    estado_robot = "HAY ALGO A LA IZQUIERDA";
  else if (rival == CRUCE_T)
    estado_robot = "T";
      else if (rival == CRUCE)
    estado_robot = " NADA ";

  SerialBT.print("sensor: ");
  SerialBT.print(estado_robot);
  SerialBT.print(" | ");
}
void ImprimirDatos(int sd, int sai, int si)
{
  SerialBT.print("   sensor_derecho: ");
  SerialBT.print(sd);
  SerialBT.print("   - sensor_frontal: ");
  SerialBT.print(sai);
  SerialBT.print("   - sensor_izquierdo: ");
  SerialBT.println(si);
}
float leerSensoresDeDistancia(){
  float lectura1 = analogRead(SEN_DIST_IZQ);
  float lectura2 = analogRead(SEN_DIST_DER);
  float distancia1 = 0;
  float distancia2 = 0;

  distancia1= 2076 / (lectura1 - 11);
  distancia2= 2076 / (lectura2 - 11);

  return distancia1, distancia2;
  }

enum ESTADOS{
    STANDBY,
    BUSQUEDA,
    ATAQUE,    
    REBOTE
  };
  switch(POSICION){
    case STANDBY{
      frenar();
      if(distancia1 || distancia 2 >= DISTANCIA_MAX) POSICION = BUSCANDO_OPONENTE;
      if(distancia1 || distancia 2 <= DISTANCIA_MAX) POSICION = OPONENTE_ENCONTRADO;
      }
    case BUSCANDO_OPONENTE{
      girar();
      if(distancia1 || distancia 2 <= DISTANCIA_MAX) POSICION = OPONENTE_ENCONTRADO;
      }
    }
//-------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  //SerialBT.begin("ESP32_Benitez"); //Nombre del Bluetooth
  AsignacionpinesMOTORES();
  asignacionPinesSensores();
  AsignacionPinesPWM();
}
void loop()
{
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  if (millis() > tiempo_actual + TICK_ULTRASONIDO)
  {
    tiempo_actual = millis();
    //  LECTURA DE LOS SENSORES
  }
  MovimientosDelRobot();
    if (SerialBT.available()) {

  if(DEBUG_DE_CASOS){
  ImprimirEstadoRobot(POSICION);
  }
  if (DEBUG_DE_SENSORES){
   ImprimirDatos(/*SENSORES*/); 
  }
    }
  
  
}