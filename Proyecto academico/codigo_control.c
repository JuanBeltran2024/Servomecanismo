// Definir variables
#define Sensor 3                 // Pin del sensor analógico
#define OutputPWM_GPIO 9           // Pin de salida PWM para el control de la hélice
#define pwmRes 12                  // Resolución del PWM (12 bits)
#define pwmMax 4095                // Valor máximo para el PWM (4095 para 12 bits)

// Variables para la conversión y salida
#define Uunits 100                 // Unidades para la salida de control (u) [mA]

// Variables de tiempo de ejecución
unsigned long pTime = 0;
unsigned long dTime = 0;
long previousMillis = 0;          // Para la función del bucle principal
long Ts = 1;                   // Tiempo de muestreo en ms
long previousMillis2 = 0;         // Para funciones auxiliares
bool up = true;
int i = 0;

// Advanced Serial Input Variables
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Variables de medición
float sensorValue = 0.0;         // Valor leído del sensor analógico
float angulo = 0.0;

// Variables de control del sistema
float Ref = 10;                // angulo de referencia        
float U_t = 0.0;                 // Salida de control (PWM)
unsigned int pwmDuty = 0;        // Ciclo de trabajo del PWM


// Variables para el controlador PID
float k_p = 1;    
float k_i = 26.4;
float k_d = 0.008;
int N = 100;
float e_n = 0.0, e_n_1 = 0.0;
float u_n = 0.0 , u_p = 0.0, u_i = 0.0, u_d = 0.0,  u_n_1_i = 0.0, u_n_1_d = 0.0, u_n_1 = 0.0;
float ts = 0.0010;


//Variables sentido de giro motor
const uint8_t PIN_IN1 = 4;  // Dirección fija
const uint8_t PIN_IN2 = 5;  // Dirección fija
bool sentidoAdelante = true; // Variable para controlar el sentido actual

//Variables medición de angulos - MODO POLLING
const uint16_t SLOTS_PER_REV  = 180;      // # de ranuras por vuelta
const bool     COUNT_ON_FALL  = true;     // true: cuenta flanco FALLING; false: RISING

int32_t slotCount = 0;                    // CAMBIADO A int32_t para permitir negativos
uint32_t lastUs = 0;                      // para antirruido
const uint32_t DEBOUNCE_US = 800;         // ignora cambios más rápidos que esto

uint8_t lastState = HIGH;                 // estado anterior del pin (inicia en HIGH por pullup)
uint32_t lastPrint = 0;

// Función para leer encoder por polling (reemplaza a la ISR)
void leerEncoder() {
  // ===== DETECCIÓN DE FLANCOS POR POLLING =====
  uint8_t currentState = digitalRead(Sensor);
  
  // Detecta cambio de estado
  if (currentState != lastState) {
    uint32_t now = micros();
    
    // Antirrebote: ignora cambios muy rápidos
    if (now - lastUs >= DEBOUNCE_US) {
      // Verifica el tipo de flanco que queremos contar
      if (COUNT_ON_FALL) {
        // Flanco descendente: lastState=HIGH, currentState=LOW
        if (lastState == HIGH && currentState == LOW) {
          // Incrementa o decrementa según el error
          if (e_n >= 0) {
            slotCount++;  // Error positivo: sumar ángulo
          } else {
            slotCount--;  // Error negativo: restar ángulo
          }
          lastUs = now;
        }
      } else {
        // Flanco ascendente: lastState=LOW, currentState=HIGH
        if (lastState == LOW && currentState == HIGH) {
          // Incrementa o decrementa según el error
          if (e_n >= 0) {
            slotCount++;  // Error positivo: sumar ángulo
          } else {
            slotCount--;  // Error negativo: restar ángulo
          }
          lastUs = now;
        }
      }
    }
    
    lastState = currentState;  // Actualiza el estado anterior
  }
}

// Función de calibración
void calibracion(void) {
    unsigned long currentMillis = millis(); // Actualizar el tiempo actual
    if (currentMillis - previousMillis >= Ts) {
        previousMillis = currentMillis;

        // Calcular ángulo ANTES de calcular el error
        // CORREGIDO: Manejo correcto de ángulos negativos y positivos
        int32_t slots = slotCount;
        
        // Ángulo puede ser negativo si hemos retrocedido
        float degPerSlot = 360.0f / (float)SLOTS_PER_REV;
        angulo = slots * degPerSlot;
        
        // Mantener el ángulo en el rango 0-360 si lo deseas
        // (comentado para permitir ver valores negativos durante desarrollo)
        // while (angulo < 0) angulo += 360.0f;
        // while (angulo >= 360.0f) angulo -= 360.0f;

        // Calcular error
        e_n = Ref - angulo;

        // Cálculo del control PID
        u_p = k_p * e_n;
        u_i = (k_i * ts * e_n_1) + u_n_1_i;
        u_d = (k_d * N * e_n) - (k_d * N * e_n_1) - (N * ts * u_n_1_d) + u_n_1_d;
        u_n = u_p + u_i + u_d;
     

        
  

        // Control PWM (siempre positivo, el sentido lo dan IN1 e IN2)
        float U_tl = min(max(u_n, 0), Uunits);
        pwmDuty = int((U_tl / Uunits) * pwmMax);
        analogWriteADJ(OutputPWM_GPIO, pwmDuty);

        // Actualizar valores anteriores
        e_n_1 = e_n;
        u_n_1_i = u_i;
        u_n_1_d = u_d;
        
        // Enviar datos al monitor serial
        Serial.print("Tiempo: ");
        Serial.print(millis());
        Serial.print(", ");
        Serial.print("angulo: ");
        Serial.print(angulo);
        Serial.print(", ");
        Serial.print("Ref: ");
        Serial.print(Ref);
        Serial.print(", ");
        Serial.print("PWM: ");
        Serial.print(((pwmDuty * 100.0) / pwmMax));
        Serial.print("%");
        Serial.print(", ");
        Serial.print("error: ");
        Serial.print(e_n);
        Serial.print(", ");
        Serial.print("U_n: ");
        Serial.println(u_n);
    }

    // Funciones de entrada serial avanzadas
    recvWithStartEndMarkers();  
    if (newData == true) {
        parseData();
        newData = false;
    }
}

// Configuración del PWM
void setupPWMadj() {
    DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
    TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
        | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
    TCCR1B = _BV(WGM13) | _BV(WGM12)
        | _BV(CS10);                    /* no prescaling */
    ICR1 = 0x0fff;                      /* TOP counter value - SETS RESOLUTION/FREQUENCY */
}

// Versión de analogWrite() de 12 bits
void analogWriteADJ(uint8_t pin, uint16_t val) {
    switch (pin) {
        case 9: OCR1A = val; break;
        case 10: OCR1B = val; break;
    }
}

// Funciones avanzadas para la entrada serial
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<'; // La entrada serial debe comenzar con este carácter
    char endMarker = '>'; // La entrada serial debe terminar con este carácter
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // Terminar la cadena
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// Función para parsear los datos recibidos
void parseData() {
    Ref = atof(receivedChars); // Convertir la entrada serial a un valor flotante y actualizar la referencia
    Serial.print("Nueva referencia: ");
    Serial.println(Ref);
}

void setup() {
    Serial.begin(115200); // Iniciar la comunicación serial a alta velocidad
    
    // Configuración de entrada analógica
    pinMode(Sensor, INPUT_PULLUP); // Configurar el pin del sensor como entrada
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);

    // Inicializar sentido del motor: adelante (IN1=LOW, IN2=HIGH)
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    sentidoAdelante = true;

    // Lee el estado inicial del pin para el encoder por polling
    lastState = digitalRead(Sensor);

    // Configuración del PWM
    setupPWMadj();
    analogWriteADJ(OutputPWM_GPIO, 0); // CORREGIDO: Iniciar en 0

    Serial.println(F("Sistema de control PID con encoder OPB800 (POLLING MODE)"));
    Serial.println(F("180 ranuras -> 2.0 grados/ranura"));
    Serial.println(F("Control bidireccional automatico segun error"));
    Serial.println(F("Envia <valor> por Serial para cambiar referencia (ej: <90>)\n"));

    delay(2000); // Esperar 2 segundos antes de iniciar
}

void loop() {
    // CORREGIDO: Eliminar control manual del sentido
    // El sentido ahora se controla ÚNICAMENTE en controlarSentidoMotor()
    
    // Leer encoder continuamente
    leerEncoder();
    
    // Ejecutar función de control
    calibracion();
}
