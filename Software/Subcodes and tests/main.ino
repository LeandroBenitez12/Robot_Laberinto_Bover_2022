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
#define DISTANCIA_MINIMA_ADE 5
#define TICK_STOP 1000
#define DISTANCIA_MINIMA_IZQ 5
#define DISTANCIA_MINIMA_DER 5
#define TICK_ULTRASONIDO 10
#define VELOCIDAD 150
#define BUZZER 23
#define BUTTON 13
// pwm DE UN MOTOR
const int freq = 5000;
const int PWMChannel = 0;
const int resolution = 8;
// pwm DE UN 2DO MOTOR
const int freq2 = 5000;
const int PWMChannel2 = 0;
const int resolution2 = 8;
// Sensores
int sensor_derecho;
int sensor_frontal;
int sensor_izquierdo;
// button
bool button_start;

int periodo = 1000;
#define PERIODO 100
unsigned long tiempo_actual = 0;
unsigned long tiempo_actual_stop = 0;
unsigned long tiempo_actual_button = 0;
// PIN_SENSORES en una matriz
int pin_sensores_ultrasonido[4][2] = {
    {ECHO1, TRIG1},
    {ECHO2, TRIG2},
    {ECHO3, TRIG3},
    {ECHO4, TRIG4},
};
// ENMASCARADO
#define FRONTAL_ACTIVO 2
#define DERECHO_ACTIVO 1
#define IZQUIERDO_ACTIVO 4

//------------------------------------------------------------------------------------------

// ESTADOS DEL SENSOR

enum ESTADO_SENSOR
{
  INICIAL,
  PASILLO,
  FRONTAL,
  FRONTAL_DERECHO,
  IZQUIERDO_FRONTAL,
  IZQUIERDO_FRONTAL_DERECHO,
  PARAR
};

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
void AsignacionPinesPWM()
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
  digitalWrite(MR1, HIGH);
  digitalWrite(MR2, LOW);
  ledcWrite(PWMChannel, VELOCIDAD);
  digitalWrite(ML1, LOW);
  digitalWrite(ML2, HIGH);
  ledcWrite(PWMChannel2, VELOCIDAD);
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
void Buzzer(){
  digitalWrite(BUZZER, HIGH);
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
int LeerButton(int button)
{
  bool estado_button;
  estado_button = digitalRead(button);
  return estado_button;
}

//-------------------------------------------------------------
void ImprimirEstadoRobot(int sensor)
{
  String estado_robot = "";
    if (sensor == INICIAL)
    estado_robot = "STANDBY";
  else if (sensor == PASILLO)
    estado_robot = "ADELANTE";
  else if (sensor == FRONTAL)
    estado_robot = "DERECHA";
  else if (sensor == FRONTAL_DERECHO)
    estado_robot = "IZQUIERDA";
  else if (sensor == IZQUIERDO_FRONTAL)
    estado_robot = "DERECHA";
  else if (sensor == PARAR)
    estado_robot = "PARAR";

  Serial.print("sensor: ");
  Serial.print(estado_robot);
  Serial.print(sensor);
  Serial.print(" | ");
}
void ImprimirDatos(int sd, int sai, int si)
{
  Serial.print("   sensor_derecho: ");
  Serial.print(sd);
  Serial.print("   - sensor_frontal: ");
  Serial.print(sai);
  Serial.print("   - sensor_izquierdo: ");
  Serial.println(si);
}
//----------------------------------------------------------------

int sensor = INICIAL;
int sensor_anterior = PARAR;
void MovimientodelRobot(int sensor)
{
  switch (sensor)
  {
  case INICIAL:
  {
    Stop();
    if (millis() > tiempo_actual_button + PERIODO)
    {
      tiempo_actual_button = millis();
      button_start = LeerButton(BUTTON);
    }
    if (button_start == true)
    {
      sensor = PASILLO;
    }
    else
    {
      sensor = INICIAL;
    }

    break;
  }
  case PASILLO:
  {
    Forward();
    if (sensor_frontal < DISTANCIA_MINIMA_ADE)
    {
      
      if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
      {
        sensor = IZQUIERDO_FRONTAL;
      }
      if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
      {
        sensor = FRONTAL_DERECHO;
      }
      if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
      {
        sensor = FRONTAL;
      }
      if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
      {
        sensor = IZQUIERDO_FRONTAL_DERECHO;
      }
    }
    break;
  }
  
case IZQUIERDO_FRONTAL:
{
  Right();
  Buzzer();
  if (sensor_frontal > DISTANCIA_MINIMA_ADE)
    sensor = PASILLO;
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
  {
    sensor = FRONTAL_DERECHO;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
  {
    sensor = FRONTAL;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
  {
    sensor = IZQUIERDO_FRONTAL_DERECHO;
  }
  break;
}
case FRONTAL_DERECHO:
{
  Left();
  Buzzer();
  if (sensor_frontal > DISTANCIA_MINIMA_ADE)
    sensor = PASILLO;
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
  {
    sensor = IZQUIERDO_FRONTAL;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
  {
    sensor = FRONTAL;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
  {
    sensor = IZQUIERDO_FRONTAL_DERECHO;
  }
}
case FRONTAL:
{
  Right();
  Buzzer();
  if (sensor_frontal > DISTANCIA_MINIMA_ADE)
  {
    sensor = PASILLO;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
  {
    sensor = IZQUIERDO_FRONTAL;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
  {
    sensor = FRONTAL_DERECHO;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
  {
    sensor = IZQUIERDO_FRONTAL_DERECHO;
  }
  break;
}
case PARAR:
{
  Stop();
    if (sensor_frontal > DISTANCIA_MINIMA_ADE)
  {
    sensor = PASILLO;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
  {
    sensor = IZQUIERDO_FRONTAL;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
  {
    sensor = FRONTAL_DERECHO;
  }
  if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
  {
    sensor = FRONTAL;
  }  
  break;
}
}
}
//-------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  AsignacionpinesMOTORES();
  asignacionPinesSensores();
  AsignacionPinesPWM();
}
void loop()
{
  if (millis() > tiempo_actual + TICK_ULTRASONIDO)
  {
    tiempo_actual = millis();
    sensor_izquierdo = LeerUltrasonidos(TRIG1, ECHO1);
    sensor_frontal = LeerUltrasonidos(TRIG2, ECHO2);
    sensor_derecho = LeerUltrasonidos(TRIG4, ECHO4);
  }
  // logica-algoritmo
  if (sensor != sensor_anterior)
  {
    MovimientodelRobot(sensor);
    sensor_anterior = sensor;
  }

  ImprimirEstadoRobot(sensor);
  ImprimirDatos(sensor_derecho, sensor_frontal, sensor_izquierdo);
}
