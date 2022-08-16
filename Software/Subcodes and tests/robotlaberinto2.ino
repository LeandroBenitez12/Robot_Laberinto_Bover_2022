#include "BluetoothSerial.h"
#include <PID_v1.h>
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
// ultrasonidos
#define ECHO1 35
#define ECHO2 33
#define ECHO3 26
#define ECHO4 14
#define TRIG1 32
#define TRIG2 25
#define TRIG3 27
#define TRIG4 12
#define DISTANCIA_MINIMA_ADE 15
#define TICK_STOP 1000
#define DISTANCIA_MINIMA_IZQ 15
#define DISTANCIA_MINIMA_DER 15
#define TICK_ULTRASONIDO 10
#define VELOCIDAD 140
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
// timer
int periodo = 1000;
#define PERIODO 100
unsigned long tiempo_actual = 0;
unsigned long tiempo_actual_stop = 0;
unsigned long tiempo_actual_button = 0;
//PID 
//Definir vairables del control PID
double Setpoint, Input, Output;
double Kp=1.2, Ki=0.19, Kd=0.001; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Accion en la interrupción 
void flash(){  
	myPID.Compute();  // Calcula la señal de control
	RunMotor(Output); // Aplica la señal de control hacia el Motor
}

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

//establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
//------------------------------------------------------------------------------------------

// ESTADOS DEL SENSOR

enum UBICACIONES
{
  INICIAL,
  PASILLO,
  FRONTAL,
  FRONTAL_DERECHO,
  IZQUIERDO_FRONTAL,
  IZQUIERDO_FRONTAL_DERECHO,
  CRUCE_T,
  CRUCE,
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
void Buzzer()
{
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
void ImprimirEstadoRobot(int ubicacion)
{
  String estado_robot = "";
  if (ubicacion == INICIAL)
    estado_robot = "STANDBY";
  else if (ubicacion == PASILLO)
    estado_robot = "PASILLO";
  else if (ubicacion == FRONTAL)
    estado_robot = "FRONTAL";
  else if (ubicacion == FRONTAL_DERECHO)
    estado_robot = "HAY ALGO A LA DERECHA";
  else if (ubicacion == IZQUIERDO_FRONTAL)
    estado_robot = "HAY ALGO A LA IZQUIERDA";

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
//----------------------------------------------------------------
// Función para accionar el giro del eje del motor
void RunMotor(double Usignal){  
	double pwmS;

	if(Usignal>=0){
     	pwmS=Usignal*10000/719-9089.0/719.0;
		shaftrev(MR1,MR2,PWMChannel,ubicacion , pwmS);
	}else{
    	pwmS=-Usignal*10000/719-9089.0/719.0;
	    shaftrev(MR1,MR2,PWMChannel,ubicacion , pwmS);
	}		
}

// Función que configura el motor que se quiere controlar
void shaftrev(int in1, int in2, int PWMChannel, int estado,int Wpulse){  
  if(estado == PASILLO ){ //backWARDS
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    ledcWrite(PWMChannel, Wpulse);
    }
  if(estado == FRONTAL){ //forWARDS
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    ledcWrite(PWMChannel, Wpulse);   
    }
}
//----------------------------------------------------------------
//Maquina de estados
int ubicacion = INICIAL;
void MovimientosDelRobot() {
  switch (ubicacion)
  {
    case INICIAL: {
         {
      ubicacion = PASILLO;
    break;
  }
        
    }
    case PASILLO:
      {
        if (sensor_frontal > DISTANCIA_MINIMA_ADE)
        {
          Forward();
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE)
        {

          if (sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
          {
            ubicacion = IZQUIERDO_FRONTAL;
          }
          if (sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
          {
            ubicacion = FRONTAL_DERECHO;
          }
          if (sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
          {
            ubicacion = FRONTAL;
          }
          if (sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
          {
            ubicacion = IZQUIERDO_FRONTAL_DERECHO;
          }
          if (sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = CRUCE_T;
        }
        }
        if (sensor_frontal > DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
            ubicacion = CRUCE;
        }
        break;
      }

    case IZQUIERDO_FRONTAL:
      {

        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          Right();
          Buzzer();
        }

        if (sensor_frontal > DISTANCIA_MINIMA_ADE)
          ubicacion = PASILLO;
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
        {
          ubicacion = FRONTAL_DERECHO;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = FRONTAL;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = CRUCE_T;
        }
        if (sensor_frontal > DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
            ubicacion = CRUCE;
        }
        break;
      }
    case FRONTAL_DERECHO:
      {
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
        {
          Left();
          Buzzer();
        }

        if (sensor_frontal > DISTANCIA_MINIMA_ADE)
          ubicacion = PASILLO;
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = IZQUIERDO_FRONTAL;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = FRONTAL;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = CRUCE_T;
        }
        if (sensor_frontal > DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
            ubicacion = CRUCE;
        }
        break;
      }
    case FRONTAL:
      {
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          Right();
          Buzzer();
        }

        if (sensor_frontal > DISTANCIA_MINIMA_ADE)
        {
          ubicacion = PASILLO;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo < DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = IZQUIERDO_FRONTAL;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
        {
          ubicacion = FRONTAL_DERECHO;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = CRUCE_T;
        }
        if (sensor_frontal > DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
            ubicacion = CRUCE;
        }
        break;
      }

    case CRUCE_T: 
    {
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          Right();
          Buzzer();
        }

        if (sensor_frontal > DISTANCIA_MINIMA_ADE)
          ubicacion = PASILLO;
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
        {
          ubicacion = FRONTAL_DERECHO;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = FRONTAL;
        }
        if (sensor_frontal > DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
            ubicacion = CRUCE;
        }
        break;
      }

    case CRUCE: {
        if (sensor_frontal > DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          Right();
          Buzzer();
        }

         if (sensor_frontal > DISTANCIA_MINIMA_ADE)
          ubicacion = PASILLO;
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho < DISTANCIA_MINIMA_DER)
        {
          ubicacion = FRONTAL_DERECHO;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = FRONTAL;
        }
        if (sensor_frontal < DISTANCIA_MINIMA_ADE && sensor_izquierdo > DISTANCIA_MINIMA_IZQ && sensor_derecho > DISTANCIA_MINIMA_DER)
        {
          ubicacion = CRUCE_T;
        }

        break;
      }
    }
    }
//-------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32_Benitez"); //Nombre del Bluetooth
  AsignacionpinesMOTORES();
  asignacionPinesSensores();
  AsignacionPinesPWM();
	//Activa el PID
	myPID.SetMode(AUTOMATIC);
    // valores maximos y minimos de salida del controlador
	myPID.SetOutputLimits(-255,255); 
	//TIempo de muestreo milisegundos para el PID
	myPID.SetSampleTime(3); 		 
}
void loop()
{

  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  if (millis() > tiempo_actual + TICK_ULTRASONIDO)
  {
    tiempo_actual = millis();
    sensor_izquierdo = LeerUltrasonidos(TRIG1, ECHO1);
    sensor_frontal = LeerUltrasonidos(TRIG2, ECHO2);
    sensor_derecho = LeerUltrasonidos(TRIG4, ECHO4);
  }
  	// Algoritmo de protección de mecanismos de posición
	if(sensor_frontal>=20){
	   shaftrev(MR1,MR2,PWMChannel,ubicacion ,0);
	}else if(sensor_frontal<=20){
	   shaftrev(MR1,MR2,PWMChannel,ubicacion ,0);
	}
	// Introduce el angulo para cerrar el lazo de control
	Input=sensor_frontal;
  MovimientosDelRobot();
    if (SerialBT.available()) {

  if(DEBUG_DE_CASOS){
  ImprimirEstadoRobot(ubicacion);
  }
  if (DEBUG_DE_SENSORES){
   ImprimirDatos(sensor_derecho, sensor_frontal, sensor_izquierdo); 
  }
    }
  
  
}