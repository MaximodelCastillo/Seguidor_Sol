#include <ESP32Servo.h>

#define LDR_X_UP        35   // Pin LDR eje x positivo
#define LDR_X_DOWN      32   // Pin LDR eje x negativo
#define LDR_Y_UP        33   // Pin LDR eje y positivo
#define LDR_Y_DOWN      25   // Pin LDR eje y negativo
#define LDR_CENTER      26   // Pin LDR central

#define SERVO_X         12  // Pin Servo horizontal
#define SERVO_Y         13  // Pin Servo vertical

#define SETPOINT_X      93  // Grados del punto de inicio del servo horizontal
#define SETPOINT_Y      90  // Grados del punto de inicio del servo vertical

#define KP              0.001 // Constante proporcional
#define KI              0.001 // Constante integral
#define KD              0.001 // Constante derivativa

#define CANTIDAD_SERVOS 2   // Cantidad de servos que se controlarán
enum servo {X, Y};          // Nombre de las posiciones de cada servo

// variables internas del controlador
unsigned long previousTime;
double lastError, acumError;

//estructura para funciones de los servos
struct data_servo {
  unsigned long previousTime;
  double lastError;
  double acumError;
  int position;
};

typedef struct data_servo DATA_SERVO;

DATA_SERVO servos[CANTIDAD_SERVOS];

Servo servo_x;
Servo servo_y;

double calcularPID(double error, DATA_SERVO servo);
int leer(int pin);
int calcular_angulo_error(int up, int center, int down);

void setup() {
  Serial.begin(9600);
  
  // Inicializar servos
  servo_x.attach(SERVO_X);
  servo_y.attach(SERVO_Y);
  servo_x.write(SETPOINT_X);
  servo_y.write(SETPOINT_Y);

  // Inicializar memoria servos
  for (int i = 0; i < CANTIDAD_SERVOS; i++) {
    servos[i].previousTime = millis();
    servos[i].lastError = 0;
    servos[i].acumError = 0;
  }
  servos[X].position = SETPOINT_X;
  servos[Y].position = SETPOINT_Y;
}

void loop() {
  //Lecturas de los sensores LDR, linea 114.
  int x_up        = leer(LDR_X_UP);
  int x_down      = leer(LDR_X_DOWN);
  int y_up        = leer(LDR_Y_UP);
  int y_down      = leer(LDR_Y_DOWN);
  int center      = leer(LDR_CENTER);

  //Serial.printf("X up: %6d, center: %6d, X down: %6d\n", x_up, center, x_down);
  
  //calcula el error de angulo con los LDRs, linea 125.
  int angulo_error_x = calcular_angulo_error(x_up, center, x_down);
  int angulo_error_y = calcular_angulo_error(y_up, center, y_down);

  servos[X].position = servos[X].position + calcularPID(angulo_error_x, servos[X]);
  servos[Y].position = servos[Y].position + calcularPID(angulo_error_y, servos[Y]);

  //bloqueo de servos encuanto a angulos
  for (int i = 0; i < CANTIDAD_SERVOS; i++) {
    servos[i].position = servos[i].position > 180 ? 180 : servos[i].position;
    servos[i].position = servos[i].position < 0   ? 0   : servos[i].position;
  }

  Serial.printf("X: Error: %6d, Posicion: %6d°\n", angulo_error_x, servos[X].position); //calcula el error del eje X y la posisión del LDR acomparación del sol
  Serial.printf("Y: Error: %6d, Posicion: %6d°\n", angulo_error_y, servos[Y].position); //calcula el error del eje Y y la posisión del LDR acomparación del sol

  servo_x.write(servos[X].position);
  servo_y.write(servos[Y].position);
  
  // Tiempo de muestreo del controlador PID,
  // debe dar suficiente tiempo de actuar a los servomotores pero no tanto que el movimiento se vea pausado.
  delay(100);
}

double calcularPID(double error, DATA_SERVO servo) {     
        unsigned long currentTime = millis();                                      // obtener el tiempo actual
        double elapsedTime = (double)(currentTime - servo.previousTime) / 1000;    // calcular el tiempo transcurrido
        
        servo.acumError += error * elapsedTime;                                    // calcular la integral del error
        double rateError = (error - servo.lastError) / elapsedTime;                // calcular la derivada del error
 
        double output = KP*error + KI*servo.acumError + KD*rateError;              // calcular la salida del PID
 
        servo.lastError = error;                                                   // almacenar error anterior
        servo.previousTime = currentTime;                                          // almacenar el tiempo anterior
 
        return output;
}

// Esta función actua como un filtro pasabajos para sacar el ruido de la señal.
int leer(int pin) {
  unsigned int reads = 0;
  
  for (int i = 0; i < 16; i++) {
    reads += analogRead(pin);
  }

  return 4096 - (reads >> 4);
}

// La idea de ésta función es medio que triangular la posición del sol en un eje, devolviendo el angulo de error, positivo up, degativo down.
int calcular_angulo_error(int up, int center, int down) {
  int tolerancia = 50;
  int angulo_max = 90 ;
  
  //calcula si tiene mas valor el up o down.
  if (abs(up - down) < tolerancia) {
    return 0;
  }
  int signo = (up > down) ? 1 : -1;
  
  // si o si, uno de los LDR extremos debe tener sombra, determinar cual es para usar como punto de referencia.
  int punto_sombra = (up < down) ? up : down;
  int punto_comparacion = (up > down) ? up : down;

  punto_comparacion = punto_comparacion - punto_sombra;
  int punto_central = center - punto_sombra;

  int maximo = punto_central + punto_comparacion;

  return ((punto_comparacion * angulo_max) / maximo) * signo;
}
