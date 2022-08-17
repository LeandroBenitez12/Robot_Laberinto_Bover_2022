
#include "BluetoothSerial.h"
// DEBUG
#define DEBUG_DE_CASOS 0
#define DEBUG_DE_SENSORES 0
#define DEBUG_PID 0
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
// Button
#define BUZZER 23
#define BUTTON 13
// const
#define DISTANCIA_MINIMA 100
#define DISTANCIA_LADOS 150
#define DISTANCIA_MAXIMA 250
#define TICK_PID 70
#define TICK_ULTRASONIDO 10
#define TICK_NOTIFICACION 500
#define TICK_DELAY 250
// veocidades motores pwm
int velocidad_derecha = 175;
int velocidad_izquierda = 175;
int velocidad_media = 175;
// pwm DE UN MOTOR
const int freq = 5000;
const int PWMChannel = 0; // LOS CANALES PWM TIENEN QUE SER DISTINTOS!!!!!!
const int resolution = 8;
// pwm DE UN 2DO MOTOR
const int freq2 = 5000;
const int PWMChannel2 = 1;
const int resolution2 = 8;
// se almacenan los datos de los sensores
int sensor_derecho;
int sensor_frontal;
int sensor_izquierdo;
// button
bool button_start;
#define PERIODO 100
unsigned long tiempo_actual = 0;
unsigned long tiempo_actual_pid = 0;
unsigned long tiempo_notificacion = 0;
unsigned long tiempo_actual_button = 0;
// PIN_SENSORES en una matriz
int pin_sensores_ultrasonido[4][2] = {
	{ECHO1, TRIG1},
	{ECHO2, TRIG2},
	{ECHO3, TRIG3},
	{ECHO4, TRIG4},
};
// PID
//------------------------------------------------------------------------------------------
double kp = 0.3;
double kd = 0;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint = 0;
double deltaError;
double PID1;

double computePID(double input)
{
	currentTime = millis();								// get current time
	elapsedTime = (double)(currentTime - previousTime); // compute time elapsed from previous computation
	error = input - setPoint;
	deltaError = error - lastError;
	double out = kp * error + kd * deltaError;
	lastError = error;			// remember current error
	previousTime = currentTime; // remember current time

	return out; // have function return the PID output
}
//------------------------------------------------------------------------------------------
// establecemos conexion bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
//------------------------------------------------------------------------------------------

void AsignacionpinesMOTORES()
{
	pinMode(MR1, OUTPUT);
	pinMode(MR2, OUTPUT);
	pinMode(ML1, OUTPUT);
	pinMode(ML2, OUTPUT);
	pinMode(BUZZER, OUTPUT);
	pinMode(BUTTON, INPUT_PULLDOWN);
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
void Motor(int estado, int pin1, int pin2, int pwm, int velocidad)
{

	ledcWrite(pwm, velocidad);

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
	Motor(estado, MR1, MR2, PWMChannel, velocidad_izquierda);
}
void MotorIzq(int estado)
{
	Motor(estado, ML1, ML2, PWMChannel2, velocidad_derecha);
}
// HACIA ADELANTE
void Left()
{
	MotorDer(AVANZAR);
	MotorIzq(AVANZAR);
}
// HACIA ATRAS
void Right()
{
	MotorDer(RETROCEDER);
	MotorIzq(RETROCEDER);
}
// GIRO SOBRE SU PROPIO EJE A LA DERECHA
void Forward()
{
	MotorDer(RETROCEDER);
	MotorIzq(AVANZAR);
}
// GIRO SOBRE SU PROPIO EJE A LA IZQUIERDA
void Backward()
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
	distancia = duracion / 5.82;
	return distancia;
}
int LeerButton(int pin_btn)
{
	return digitalRead(pin_btn);
}

//-------------------------------------------------------------
void ImprimirEstadoRobot(int ubicacion)
{
	String estado_robot = "";
	if (ubicacion == INICIAL)
	{
		estado_robot = "sta off";
	}
	else if (ubicacion == PASILLO)
	{
		estado_robot = "segui noma";
	}
	else if (ubicacion == DESVIO_IZQUIERDA)
	{
		estado_robot = "DOBLAR a la IZQUIERDA";
	}
	else if (ubicacion == DESVIO_DERECHA)
	{
		estado_robot = " doblar a la derecha";
	}
	else if (ubicacion == CRUCE_T)
	{
		estado_robot = "elegi un lado";
	}
	else if (ubicacion == CRUCE)
	{
		estado_robot = " para donde quieras rey ";
	}
	else if (ubicacion == CALLEJON)
	{
		estado_robot = " te encerraste";
	}
	else if (ubicacion == CRUCE_OBLIGATORIO_IZQ)
	{
		estado_robot = " segui derecho";
	}
	else if (ubicacion == CRUCE_OBLIGATORIO_DER)
	{
		estado_robot = " doblaaa";
	}
	else if (ubicacion == TRABADO)
	{
		estado_robot = " me trabe";
	}

	SerialBT.print("State: ");
	SerialBT.print(estado_robot);
	SerialBT.print(" | ");
	SerialBT.print(button_start);
	SerialBT.print(" | ");
}
void ImprimirDatos(int sd, int sai, int si)
{
	SerialBT.print(" sI:");
	SerialBT.print(sd);
	SerialBT.print("- sF: ");
	SerialBT.print(sai);
	SerialBT.print(" - sD: ");
	SerialBT.print(si);
}
void ImprimirPID()
{
	SerialBT.println(PID1);
	SerialBT.print(" | vel_der");
	SerialBT.print(velocidad_derecha);
	SerialBT.print(" | vel_izq ");
	SerialBT.print(velocidad_izquierda);
}
//----------------------------------------------------------------
// Moviemientos del robot
enum UBICACIONES
{
	INICIAL,
	PASILLO,
	DESVIO_IZQUIERDA,
	DESVIO_DERECHA,
	CALLEJON,
	CRUCE_T,
	CRUCE,
	TRABADO,
	CRUCE_OBLIGATORIO_DER,
	CRUCE_OBLIGATORIO_IZQ
};

//------------------------------------------------------------------------------------------

// Maquina de estados
int ubicacion = INICIAL;
void MovimientosDelRobot()
{
	switch (ubicacion)
	{
	case INICIAL:
	{
		if (millis() > tiempo_actual_button + PERIODO)
		{
			tiempo_actual_button = millis();
			button_start = LeerButton(BUTTON);
		}
		if (button_start)
		{
			delay(3000);
			ubicacion = PASILLO;
		}
		else
		{
			Stop();
		}

		break;
	}

	case PASILLO:
	{
		// pid

		if (sensor_frontal > DISTANCIA_MINIMA && sensor_izquierdo <= DISTANCIA_LADOS && sensor_derecho <= DISTANCIA_LADOS)
		{

			velocidad_derecha = 200;
			velocidad_izquierda = 200;
			Forward();
		}
		else
		{
			velocidad_derecha = 170;
			velocidad_izquierda = 170;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA)
		{

			if (sensor_izquierdo < DISTANCIA_LADOS && sensor_derecho > DISTANCIA_LADOS)
			{
				ubicacion = DESVIO_DERECHA;
			}
			if (sensor_izquierdo > DISTANCIA_LADOS && sensor_derecho < DISTANCIA_LADOS)
			{
				ubicacion = DESVIO_IZQUIERDA;
			}
			if (sensor_izquierdo > DISTANCIA_LADOS && sensor_derecho > DISTANCIA_LADOS)
			{
				ubicacion = CRUCE_T;
			}
			if (sensor_izquierdo < DISTANCIA_LADOS && sensor_derecho < DISTANCIA_LADOS)
			{
				ubicacion = CALLEJON;
			}
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE;
		}
		if (sensor_frontal > 20000 || sensor_izquierdo > 2000 || sensor_derecho > 2000)
		{
			ubicacion = TRABADO;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}

		break;
	}

	case DESVIO_DERECHA:
	{
		Right();
		delay(TICK_DELAY);

		if (sensor_frontal > DISTANCIA_MINIMA)
		{
			ubicacion = PASILLO;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = DESVIO_IZQUIERDA;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_T;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE;
		}
		if (sensor_frontal > 20000 || sensor_izquierdo > 2000 || sensor_derecho > 2000)
		{
			ubicacion = TRABADO;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}

		if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CALLEJON;
		}
		break;
	}
	case DESVIO_IZQUIERDA:
	{

		Left();
		delay(TICK_DELAY);

		if (sensor_frontal > DISTANCIA_MINIMA)
		{
			ubicacion = PASILLO;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = DESVIO_DERECHA;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_T;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE;
		}
		if (sensor_frontal > 20000 || sensor_izquierdo > 2000 || sensor_derecho > 2000)
		{
			ubicacion = TRABADO;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}

		if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CALLEJON;
		}
		break;
	}
	case CRUCE_T:
	{

		Left();
		delay(TICK_DELAY);

		if (sensor_frontal > DISTANCIA_MINIMA)
		{
			{
				ubicacion = PASILLO;
			}
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = DESVIO_IZQUIERDA;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = DESVIO_DERECHA;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE;
		}
		if (sensor_frontal > 20000 || sensor_izquierdo > 2000 || sensor_derecho > 2000)
		{
			ubicacion = TRABADO;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}

		if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CALLEJON;
		}
		break;
	}

	case CRUCE:
	{

		Left();
		delay(TICK_DELAY);
		if (sensor_frontal > DISTANCIA_MINIMA)
		{
			ubicacion = PASILLO;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = DESVIO_IZQUIERDA;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = DESVIO_DERECHA;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA && sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_T;
		}
		if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CALLEJON;
		}
		if (sensor_frontal > 20000 || sensor_izquierdo > 2000 || sensor_derecho > 2000)
		{
			ubicacion = TRABADO;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}

		break;
	}
	case CALLEJON:
	{
		if (sensor_frontal < DISTANCIA_MINIMA && sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			Left();
			delay(500);
		}
		if (sensor_frontal > DISTANCIA_MINIMA)
		{
			ubicacion = PASILLO;
		}

		if (sensor_frontal <= DISTANCIA_MINIMA)
		{

			if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
			{
				ubicacion = DESVIO_DERECHA;
			}
			if (sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
			{
				ubicacion = DESVIO_IZQUIERDA;
			}
			if (sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
			{
				ubicacion = CRUCE_T;
			}
			if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
			{
				ubicacion = CALLEJON;
			}
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE;
		}
		if (sensor_frontal > 20000 || sensor_izquierdo > 2000 || sensor_derecho > 2000)
		{
			ubicacion = TRABADO;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}

		break;
	}
	case TRABADO:
	{

		if (sensor_frontal > 15000 || sensor_derecho > 15000 || sensor_izquierdo > 15000)
		{
			velocidad_derecha = 250;
			velocidad_izquierda = 250;
			Backward();
			delay(TICK_DELAY);
			Left();
			delay(TICK_DELAY);
		}
		else
		{
			velocidad_derecha = velocidad_media;
			velocidad_izquierda = velocidad_media;
		}
		if (sensor_frontal > DISTANCIA_MINIMA && sensor_izquierdo <= DISTANCIA_LADOS && sensor_derecho <= DISTANCIA_LADOS)
		{
			ubicacion = PASILLO;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA)
		{

			if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
			{
				ubicacion = DESVIO_DERECHA;
			}
			if (sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
			{
				ubicacion = DESVIO_IZQUIERDA;
			}
			if (sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
			{
				ubicacion = CRUCE_T;
			}
			if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
			{
				ubicacion = CALLEJON;
			}
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}
		break;
	}
	case CRUCE_OBLIGATORIO_DER:
	{
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			Forward();
		}
		if (sensor_frontal > DISTANCIA_MINIMA && sensor_izquierdo <= DISTANCIA_LADOS && sensor_derecho <= DISTANCIA_LADOS)
		{
			ubicacion = PASILLO;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA)
		{

			if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
			{
				ubicacion = DESVIO_DERECHA;
			}
			if (sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
			{
				ubicacion = DESVIO_IZQUIERDA;
			}
			if (sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
			{
				ubicacion = CRUCE_T;
			}
			if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
			{
				ubicacion = CALLEJON;
			}
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE;
		}
		if (sensor_frontal > 20000 || sensor_izquierdo > 2000 || sensor_derecho > 2000)
		{
			ubicacion = TRABADO;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}

		break;
	}

	case CRUCE_OBLIGATORIO_IZQ:
	{
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			Left();
			delay(TICK_DELAY);
		}
		if (sensor_frontal > DISTANCIA_MINIMA && sensor_izquierdo <= DISTANCIA_LADOS && sensor_derecho <= DISTANCIA_LADOS)
		{
			ubicacion = PASILLO;
		}
		if (sensor_frontal <= DISTANCIA_MINIMA)
		{

			if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
			{
				ubicacion = DESVIO_DERECHA;
			}
			if (sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
			{
				ubicacion = DESVIO_IZQUIERDA;
			}
			if (sensor_izquierdo > DISTANCIA_MINIMA && sensor_derecho > DISTANCIA_MINIMA)
			{
				ubicacion = CRUCE_T;
			}
			if (sensor_izquierdo < DISTANCIA_MINIMA && sensor_derecho < DISTANCIA_MINIMA)
			{
				ubicacion = CALLEJON;
			}
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MAXIMA)
		{
			ubicacion = CRUCE;
		}
		if (sensor_frontal > 20000 || sensor_izquierdo > 2000 || sensor_derecho > 2000)
		{
			ubicacion = TRABADO;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}
		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo > DISTANCIA_MAXIMA && sensor_derecho < DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_IZQ;
		}

		if (sensor_frontal > DISTANCIA_MAXIMA && sensor_izquierdo < DISTANCIA_MAXIMA && sensor_derecho > DISTANCIA_MINIMA)
		{
			ubicacion = CRUCE_OBLIGATORIO_DER;
		}

		break;
	}
	}
}
//-------------------------------------------------------------
void setup()
{
	Serial.begin(9600);
	SerialBT.begin("Laberinto_BOver"); // Nombre del Bluetooth
	AsignacionpinesMOTORES();
	asignacionPinesSensores();
	AsignacionPinesPWM();
}  

void loop()
{
	if (Serial.available())
	{
		SerialBT.write(Serial.read());
	}

	if (millis() > tiempo_actual + TICK_ULTRASONIDO)
	{
		tiempo_actual = millis();
		sensor_izquierdo = LeerUltrasonidos(TRIG4, ECHO4);
		sensor_frontal = LeerUltrasonidos(TRIG2, ECHO2);
		sensor_derecho = LeerUltrasonidos(TRIG1, ECHO1);
	}
	// pid

	double Input = sensor_derecho - sensor_izquierdo;

	if (millis() > tiempo_actual_pid + TICK_PID)
	{
		tiempo_actual_pid = millis();
		PID1 = computePID(Input);
	}

	if (PID1 > 30)
		PID1 = 30;
	if (PID1 < -30)
		PID1 = -30;
 
	MovimientosDelRobot();

	if (SerialBT.available())
	{
		if (millis() > tiempo_notificacion + TICK_NOTIFICACION)
		{
			tiempo_notificacion = millis();

			if (DEBUG_DE_CASOS)
			{
				ImprimirEstadoRobot(ubicacion);
			}

			if (DEBUG_DE_SENSORES)
			{
				ImprimirDatos(sensor_derecho, sensor_frontal, sensor_izquierdo);
			}

			if (DEBUG_PID)
			{
				ImprimirPID();
			}
		}
	}
}
