// Definir variables
#define Sensor_1 2                  // Pin del encoder motor 1
#define Sensor_2 3                  // Pin del encoder motro 2
#define OutputPWM_GPIO_1 9          // Pin de salida PWM para el control motor 1
#define IN_1_1 4  
#define IN_2_1 5         
#define OutputPWM_GPIO_2 10           // Pin de salida PWM para el control motor 2
#define IN_1_2 4  
#define IN_2_2 5 
#define pwmRes 12                  // Resolución del PWM (12 bits)
#define pwmMax 4095                // Valor máximo para el PWM (4095 para 12 bits)

// Variables para la conversión y salida
#define Uunits 100                 // Unidades para la salida de control (u) [mA]

// Variables de tiempo de ejecución
unsigned long pTime = 0;
unsigned long dTime = 0;
long previousMillis = 0;          // Para la función del bucle principal
long Ts = 1000;                   // Tiempo de muestreo en ms
long previousMillis2 = 0;         // Para funciones auxiliares
bool up = true;
int i = 0;
float ts = 0.010;

// Advanced Serial Input Variables
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Variables de medición
float encoder_1 = 0.0;         // Valor leído del sensor analógico
float encoder_2 = 0.0;   
float angulo_1 = 0.0;
float angulo_2 = 0.0;

// Variables de control del sistema
float Ref_1 = 0;                // angulo de referencia motor 1
float Ref_1_Fut = 0;
float Ref_2 = 20;                // angulo de referencia motor 2   
float Ref_2_Fut = 0 ;    
float U_t_1 = 0.0;                 // Salida de control motor 1 (PWM)
float U_t_2 = 0.0;                 // Salida de control motro 2 (PWM)
unsigned int pwmDuty_1 = 0;        // Ciclo de trabajo del PWM motor 1
unsigned int pwmDuty_1_1 = 0;
unsigned int pwmDuty_2 = 0;        // Ciclo de trabajo del PWM motor 2
unsigned int pwmDuty_2_2 = 0;

// Variables para el controlador PID
float k_p_1 = 0.3; // proporcional motor 1 
float k_p_2 = 0.3; // Proporcional motor 2  
float e_n_1 = 0.0, e_n_1_1 = 0.0; //error control motor 1
float e_n_2 = 0.0, e_n_2_1 = 0.0; //error control motor 2
float u_n_1 = 0.0 , u_p_1 = 0.0, u_n_1_1 = 0.0; // motor 1
float u_n_2 = 0.0 , u_p_2 = 0.0, u_n_2_1 = 0.0; // motor 2

// Variables de control
unsigned long pwmStartTime = 0;   // Variable para el tiempo de inicio del PWM constante
unsigned long pwmDuration = 40000; // Duración del PWM constante en milisegundos (3 segundos en este caso)
bool pwmConstant = true;    

// Variables medida sensor
const uint16_t SLOTS_PER_REV_1  = 180;      // # de ranuras por vuelta
const uint16_t SLOTS_PER_REV_2  = 180;
const bool     COUNT_ON_FALL_1  = true;     // true: cuenta flanco FALLING; false: RISING
const bool     COUNT_ON_FALL_2  = true; 
const uint32_t PRINT_EVERY_MS = 100;      // período de impresión

volatile uint32_t slotCount_1 = 0;          // ranuras acumuladas (flancos contados)
volatile uint32_t slotCount_2 = 0;
volatile uint32_t lastUs_1 = 0;             // para antirruido
volatile uint32_t lastUs_2 = 0; 
const uint32_t DEBOUNCE_US = 150;         // ignora cambios más rápidos que esto

uint32_t lastPrint = 0;

// Función de calibración
void calibracion(void) {
    unsigned long currentMillis = millis(); // Actualizar el tiempo actual
    if (currentMillis - previousMillis >= Ts) {
        previousMillis = currentMillis;

        e_n_1 = Ref_1 - angulo_1;
        e_n_2 = Ref_2 - angulo_2;

        //ENCODER 1
         noInterrupts();
            uint32_t slots_1 = slotCount_1;
         interrupts();

         // Ángulo absoluto encoder 1 (0.. <360)
           uint32_t slotsMod_1 = (SLOTS_PER_REV_1 == 0) ? 0 : (slots_1 % SLOTS_PER_REV_1);
           float degPerSlot_1 = 360.0 / (float)SLOTS_PER_REV_1;
            angulo_1 = slotsMod_1 * degPerSlot_1;
         // Vueltas completas encoder 1
             uint32_t revs_1 = (SLOTS_PER_REV_1 == 0) ? 0 : (slots_1 / SLOTS_PER_REV_1);


         noInterrupts();
            uint32_t slots_2 = slotCount_2;
         interrupts();

         // Ángulo absoluto encoder 1 (0.. <360)
           uint32_t slotsMod_2 = (SLOTS_PER_REV_2 == 0) ? 0 : (slots_2 % SLOTS_PER_REV_2);
           float degPerSlot_2 = 360.0 / (float)SLOTS_PER_REV_2;
            angulo_2 = slotsMod_2 * degPerSlot_2;
         // Vueltas completas
             uint32_t revs_2 = (SLOTS_PER_REV_2 == 0) ? 0 : (slots_2 / SLOTS_PER_REV_2);

        
          u_p_1 = k_p_1*e_n_1;
          u_n_1 = u_p_1;

          u_p_2 = k_p_2*e_n_2;
          u_n_2 = u_p_2;

        if (Ref_1>Ref_1_Fut){
        digitalWrite(IN_1_1, LOW);
        digitalWrite(IN_2_1, HIGH);
        float U_tl_1 = min(max((u_n_1), 0), Uunits); // Control motor 1 saturado
        pwmDuty_1 = int((U_tl_1 / Uunits) * pwmMax); // Convertir a ciclo de trabajo PWM
        analogWriteADJ(OutputPWM_GPIO_1, pwmDuty_1); // Escribir el valor de PWM en el pin
        } else {
        digitalWrite(IN_1_1, HIGH);
        digitalWrite(IN_2_1, LOW);
        float U_tl_1 = min(max((u_n_1), 0), Uunits); // Control motor 1 saturado
        pwmDuty_1 = int((U_tl_1 / Uunits) * pwmMax); // Convertir a ciclo de trabajo PWM
        analogWriteADJ(OutputPWM_GPIO_1, pwmDuty_1); // Escribir el valor de PWM en el pin
        }


        if (Ref_2>Ref_2_Fut){
        digitalWrite(IN_1_2, HIGH);
        digitalWrite(IN_2_2, LOW);
        float U_tl_2 = min(max((u_n_2), 0), Uunits); // Control motor 2 saturado
        pwmDuty_2 = int((U_tl_2 / Uunits) * pwmMax); // Convertir a ciclo de trabajo PWM
        analogWriteADJ(OutputPWM_GPIO_2, pwmDuty_2); // Escribir el valor de PWM en el pin
        }else {
        digitalWrite(IN_1_2, LOW);
        digitalWrite(IN_1_2, HIGH);
        float U_tl_2 = min(max((u_n_2), 0), Uunits); // Control motor 2 saturado
        pwmDuty_2 = int((U_tl_2 / Uunits) * pwmMax); // Convertir a ciclo de trabajo PWM
        analogWriteADJ(OutputPWM_GPIO_2, pwmDuty_2); // Escribir el valor de PWM en el pin
        }
        
           e_n_1_1 = e_n_1;
           e_n_2_1 = e_n_2;
                

        // Enviar datos al monitor serial
        Serial.print("Tiempo: ");
        Serial.print(millis()/100);
        Serial.print(", ");
        Serial.print("angulos (1)-(2): ");
        Serial.print(angulo_1);
        Serial.print(", ");
        Serial.println(angulo_2);
      //  Serial.print(", ");
      //  Serial.print("PWM (1)-(2): ");
      //  Serial.print(((pwmDuty_1 * 100.0) / pwmMax));
      //  Serial.print(((pwmDuty_2 * 100.0) / pwmMax));
      //  Serial.print("%");
      //  Serial.print(", ");
      //  Serial.print("error (1)-(2): ");
      //  Serial.print(e_n_1);
      //  Serial.print(e_n_2);
      //  Serial.print(", ");
      //  Serial.print("U_n (1)-(2): ");
      //  Serial.print(u_n_1);
      //  Serial.println(u_n_2);
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

void isrSlot1() {
  uint32_t now_1 = micros();
  if (now_1 - lastUs_1 < DEBOUNCE_US) return; // filtro rápido contra rebotes/ruido
  slotCount_1++;
  lastUs_1 = now_1;
}

void isrSlot2() {
  uint32_t now_2 = micros();
  if (now_2 - lastUs_2 < DEBOUNCE_US) return; // filtro rápido contra rebotes/ruido
  slotCount_2++;
  lastUs_2 = now_2;
}


// Función para parsear los datos recibidos
void parseData() {
    Ref_1 = atof(receivedChars); // Convertir la entrada serial a un valor flotante y actualizar la referencia
    Ref_2 = atof(receivedChars); // Convertir la entrada serial a un valor flotante y actualizar la referencia
}

void setup() {
    Serial.begin(9600); // Iniciar la comunicación serial a alta velocidad
    
    // Configuración de entrada analógica
    pinMode(Sensor_1, INPUT_PULLUP); // Configurar el pin del encoder_1 como entrada
    pinMode(Sensor_2, INPUT_PULLUP); // Configurar el pin del Encoder_2 como entrada
    pinMode(IN_1_1, OUTPUT); 
    pinMode(IN_2_1, OUTPUT); 
    pinMode(IN_1_2, OUTPUT); 
    pinMode(IN_2_2, OUTPUT); 

       // Elige flanco
  if (COUNT_ON_FALL_1) {
    attachInterrupt(digitalPinToInterrupt(Sensor_1), isrSlot1, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(Sensor_1), isrSlot1, RISING);
  }

    if (COUNT_ON_FALL_2) {
    attachInterrupt(digitalPinToInterrupt(Sensor_2), isrSlot2, FALLING);
  } else {
    attachInterrupt(digitalPinToInterrupt(Sensor_2), isrSlot2, RISING);
  }

    // Configuración del PWM
    setupPWMadj();
    analogWriteADJ(OutputPWM_GPIO_1, pwmDuty_1);
     analogWriteADJ(OutputPWM_GPIO_2, pwmDuty_2);

    delay(5000); // Esperar 5 segundos antes de iniciar el monitor serial
}



void loop() {
    calibracion();
}
