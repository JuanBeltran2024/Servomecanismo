/*
 * ============================================================================
 * CONTROL P 2R - TRAYECTORIA TRÉBOL (ENCODER ÓPTICO 1 CANAL, SIN CERO ABSOLUTO)
 * - Cuenta incremental con signo (usa el sentido que tú mandas al motor)
 * - Corrige desfase inicial: 0° de referencia a la derecha; mecanismo inicia a 180° CCW
 * - Lectura de trayectoria desde PROGMEM (AVR)
 * ============================================================================
 */

#include <Arduino.h>
#include <avr/pgmspace.h>  // para leer PROGMEM en AVR

// ===== TELEMETRÍA (opcional) =====
#define ENABLE_TELEMETRY 1
const uint8_t TEL_DIV = 5;      // imprime 1 de cada N ciclos (si Ts=100 ms → ~2 Hz)
static uint8_t tel_cnt = 0;

// ===== PINES =====
#define Sensor_1 2                  // Encoder 1 (INT0)
#define Sensor_2 3                  // Encoder 2 (INT1)
#define OutputPWM_GPIO_1 9          // PWM motor 1 (Timer1 OCR1A)
#define IN_1_1 4
#define IN_2_1 5
#define OutputPWM_GPIO_2 10         // PWM motor 2 (Timer1 OCR1B)
#define IN_1_2 7
#define IN_2_2 8

// ===== PWM / CONTROL =====
#define pwmRes 12
#define pwmMax 4095
#define Uunits 100

const unsigned int PWM_MIN_MOTOR_1 = (unsigned int)(0.0f * 4095.0f / 255.0f);
const unsigned int PWM_MIN_MOTOR_2 = (unsigned int)(0.0f * 4095.0f / 255.0f);

float k_p_1 = 0.43f;
float k_p_2 = 0.023f;

// ===== TIEMPOS =====
unsigned long previousMillis = 0;
const long Ts = 100;                 // ms
uint32_t lastTrajUpdate = 0;
const uint32_t TRAJ_UPDATE_MS = 100; // ms

// ===== ENCODER (1 canal): antirruido y conteo bruto =====
const uint16_t SLOTS_PER_REV_1  = 180;
const uint16_t SLOTS_PER_REV_2  = 180;
const bool     COUNT_ON_FALL_1  = true;
const bool     COUNT_ON_FALL_2  = true;
const uint32_t DEBOUNCE_US      = 800;

volatile uint32_t pulseCount_1 = 0;   // pulsos crudos (sin signo)
volatile uint32_t pulseCount_2 = 0;
volatile uint32_t lastUs_1 = 0;
volatile uint32_t lastUs_2 = 0;

// ===== ESTIMACIÓN DE ÁNGULOS (con signo) =====
// CERO FÍSICO: 0° de referencia está a la derecha (+X). El mecanismo inicia a 180° CCW.
const float THETA0_DEG_1 = 180.0f;    // desfase inicial conj. 1
const float THETA0_DEG_2 = 180.0f;    // desfase inicial conj. 2

// Acumuladores de conteo con signo (ángulo incremental)
int32_t signedCount_1 = 0;
int32_t signedCount_2 = 0;
uint32_t prevPulse_1  = 0;
uint32_t prevPulse_2  = 0;

// Sentido de giro que ESTAMOS ordenando al motor: +1 = sentido positivo, -1 = negativo
int8_t dir_cmd_1 = +1;
int8_t dir_cmd_2 = +1;

// Ángulos medidos (grados y rad)
float angulo_1 = 0.0f;     // 0..360 envuelto con offset aplicado
float angulo_2 = 0.0f;
float angulo_1_rad = 0.0f;
float angulo_2_rad = 0.0f;

// ===== TRAYECTORIA (radianes) =====
const float STEP_DEG = 2.0f;
const float STEP_RAD = STEP_DEG * (PI / 180.0f);
const uint16_t TRAJECTORY_SIZE = 108;

const float trajectory_theta1[108] PROGMEM = {
  0.174533f, -0.034907f, 0.000000f, 0.034907f, 0.034907f, 0.034907f, 0.034907f, 0.034907f,
  0.069813f, 0.104720f, 0.104720f, 0.139626f, 0.139626f, 0.174533f, 0.174533f, 0.209440f,
  0.209440f, 0.209440f, 0.244346f, 0.244346f, 0.279253f, 0.314159f, 0.314159f, 0.314159f,
  0.314159f, 0.279253f, 0.279253f, 0.279253f, 0.244346f, 0.244346f, 0.209440f, 0.209440f,
  0.209440f, 0.174533f, 0.174533f, 0.174533f, 0.209440f, 0.244346f, 0.279253f, 0.314159f,
  0.314159f, 0.314159f, 0.314159f, 0.314159f, 0.279253f, 0.279253f, 0.244346f, 0.244346f,
  0.209440f, 0.174533f, 0.174533f, 0.139626f, 0.104720f, 0.069813f, 0.069813f, 0.034907f,
  0.034907f, 0.034907f, 0.034907f, 0.034907f, 0.034907f, 0.000000f, 0.000000f, 0.000000f,
  -0.034907f, -0.034907f, -0.069813f, -0.104720f, -0.139626f, -0.174533f, -0.174533f, -0.174533f,
  -0.174533f, -0.174533f, -0.174533f, -0.174533f, -0.209440f, -0.244346f, -0.279253f, -0.314159f,
  -0.349066f, -0.383972f, -0.418879f, -0.418879f, -0.418879f, -0.383972f, -0.383972f, -0.349066f,
  -0.314159f, -0.279253f, -0.279253f, -0.244346f, -0.209440f, -0.209440f, -0.174533f, -0.174533f,
  -0.139626f, -0.139626f, -0.139626f, -0.139626f, -0.139626f, -0.139626f, -0.104720f, -0.104720f,
  -0.104720f, -0.069813f, -0.069813f, -0.034907f
};

const float trajectory_theta2[108] PROGMEM = {
  2.129302f, 1.466077f, 1.466077f, 1.466077f, 1.500983f, 1.535890f, 1.570796f, 1.605703f,
  1.605703f, 1.570796f, 1.535890f, 1.535890f, 1.500983f, 1.500983f, 1.466077f, 1.466077f,
  1.431170f, 1.396263f, 1.396263f, 1.361357f, 1.361357f, 1.361357f, 1.396263f, 1.431170f,
  1.466077f, 1.466077f, 1.500983f, 1.535890f, 1.570796f, 1.605703f, 1.605703f, 1.640609f,
  1.675516f, 1.675516f, 1.710423f, 1.745329f, 1.745329f, 1.745329f, 1.745329f, 1.745329f,
  1.780236f, 1.815142f, 1.850049f, 1.884956f, 1.884956f, 1.919862f, 1.919862f, 1.954769f,
  1.954769f, 1.954769f, 1.989675f, 1.989675f, 1.989675f, 1.989675f, 2.024582f, 2.024582f,
  2.059489f, 2.094395f, 2.129302f, 2.164208f, 2.199115f, 2.199115f, 2.234021f, 2.268928f,
  2.268928f, 2.303835f, 2.303835f, 2.303835f, 2.303835f, 2.303835f, 2.268928f, 2.234021f,
  2.199115f, 2.164208f, 2.129302f, 2.094395f, 2.094395f, 2.094395f, 2.094395f, 2.094395f,
  2.094395f, 2.094395f, 2.094395f, 2.059489f, 2.024582f, 1.989675f, 1.954769f, 1.954769f,
  1.919862f, 1.919862f, 1.884956f, 1.884956f, 1.884956f, 1.850049f, 1.850049f, 1.815142f,
  1.815142f, 1.780236f, 1.745329f, 1.710423f, 1.675516f, 1.640609f, 1.605703f, 1.570796f,
  1.535890f, 1.535890f, 1.500983f, 1.466077f
};

uint16_t trajectory_index = 0;
float Ref_1 = 0.0f, Ref_2 = 0.0f;       // referencias actuales (rad)
float Ref_1_Fut = 0.0f, Ref_2_Fut = 0.0f;

// ===== UTILIDADES =====
static inline float clampf(float x, float a, float b){ return x < a ? a : (x > b ? b : x); }
static inline float wrapToPi(float a){ while(a > PI) a -= 2.0f*PI; while(a <= -PI) a += 2.0f*PI; return a; }
static inline float wrapDeg0_360(float d){ while(d >= 360.0f) d -= 360.0f; while(d < 0.0f) d += 360.0f; return d; }
static inline float angDiffDeg(float ref_deg, float y_deg){
  // diferencia angular envuelta a [-180,180] en grados
  float e = ref_deg - y_deg;
  while (e > 180.0f)  e -= 360.0f;
  while (e <= -180.0f) e += 360.0f;
  return e;
}
// PROGMEM read
static inline float pgm_read_float_idx(const float* base, uint16_t idx){
  return pgm_read_float_near(base + idx);
}

// ===== PWM Timer1 (12 bits en 9/10) =====
void setupPWMadj() {
  DDRB |= _BV(PB1) | _BV(PB2);                  // pins 9/10 salida
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13)  | _BV(WGM12)  | _BV(CS10); // no prescaler
  ICR1 = 0x0fff;                                 // TOP = 4095
}
void analogWriteADJ(uint8_t pin, uint16_t val){
  if (val > pwmMax) val = pwmMax;
  switch(pin){ case 9: OCR1A = val; break; case 10: OCR1B = val; break; }
}

// ===== ISRs: cuentan pulsos crudos (sin dirección) =====
void isrSlot1(){
  uint32_t now_1 = micros();
  if (now_1 - lastUs_1 < DEBOUNCE_US) return;
  pulseCount_1++;
  lastUs_1 = now_1;
}
void isrSlot2(){
  uint32_t now_2 = micros();
  if (now_2 - lastUs_2 < DEBOUNCE_US) return;
  pulseCount_2++;
  lastUs_2 = now_2;
}

// ===== BUCLE DE CONTROL =====
void calibracion(void){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis < Ts) return;
  previousMillis = currentMillis;

  // --- Referencias objetivo desde trayectoria (PROGMEM) ---
  float theta1_target = pgm_read_float_idx(trajectory_theta1, trajectory_index);
  float theta2_target = pgm_read_float_idx(trajectory_theta2, trajectory_index);

  // Avance de referencia por pasos de 2° (rad)
  float err1r = wrapToPi(theta1_target - Ref_1);
  if (fabsf(err1r) > 0.0001f){
    float step1 = (fabsf(err1r) >= STEP_RAD) ? (err1r > 0.0f ? STEP_RAD : -STEP_RAD) : err1r;
    Ref_1 = wrapToPi(Ref_1 + step1);
  }
  float err2r = wrapToPi(theta2_target - Ref_2);
  if (fabsf(err2r) > 0.0001f){
    float step2 = (fabsf(err2r) >= STEP_RAD) ? (err2r > 0.0f ? STEP_RAD : -STEP_RAD) : err2r;
    Ref_2 = wrapToPi(Ref_2 + step2);
  }

  // Avance de índice cuando ambos alcanzan su paso y pasó el tiempo mínimo
  bool ok1 = (fabsf(err1r) < STEP_RAD);
  bool ok2 = (fabsf(err2r) < STEP_RAD);
  if (ok1 && ok2 && (currentMillis - lastTrajUpdate >= TRAJ_UPDATE_MS)){
    lastTrajUpdate = currentMillis;
    trajectory_index = (trajectory_index + 1) % TRAJECTORY_SIZE;
  }

  // --- Lectura de pulsos y construcción de ángulo con signo ---
  uint32_t p1, p2;
  noInterrupts();
    p1 = pulseCount_1;
    p2 = pulseCount_2;
  interrupts();

  uint32_t d1 = p1 - prevPulse_1;   // nuevos pulsos desde el último ciclo
  uint32_t d2 = p2 - prevPulse_2;
  prevPulse_1 = p1;
  prevPulse_2 = p2;

  // Integro con el signo que YO ordeno al motor (single-channel no puede medir dirección)
  signedCount_1 += (int32_t)dir_cmd_1 * (int32_t)d1;
  signedCount_2 += (int32_t)dir_cmd_2 * (int32_t)d2;

  // Pulsos -> grados; aplico offset inicial + wrap [0,360)
  const float degPerSlot_1 = 360.0f / (float)SLOTS_PER_REV_1;
  const float degPerSlot_2 = 360.0f / (float)SLOTS_PER_REV_2;

  angulo_1 = wrapDeg0_360(THETA0_DEG_1 + (float)signedCount_1 * degPerSlot_1);
  angulo_2 = wrapDeg0_360(THETA0_DEG_2 + (float)signedCount_2 * degPerSlot_2);
  angulo_1_rad = angulo_1 * PI / 180.0f;
  angulo_2_rad = angulo_2 * PI / 180.0f;

  // --- Error angular por ruta corta (en grados) ---
  float Ref_1_deg = wrapDeg0_360(Ref_1 * 180.0f / PI);
  float Ref_2_deg = wrapDeg0_360(Ref_2 * 180.0f / PI);

  float e_n_1 = angDiffDeg(Ref_1_deg, angulo_1);  // [-180,180]
  float e_n_2 = angDiffDeg(Ref_2_deg, angulo_2);

  // --- Control P ---
  float u_n_1 = k_p_1 * e_n_1;
  float u_n_2 = k_p_2 * e_n_2;

  // --- Motor 1 ---
  unsigned int pwm_control_1 = 0;
  if (u_n_1 >= 0){
    // sentido positivo → defino convención de "positivo"
    digitalWrite(IN_1_1, HIGH); digitalWrite(IN_2_1, LOW);
    dir_cmd_1 = +1;
    float U1 = clampf(u_n_1, 0.0f, (float)Uunits);
    pwm_control_1 = (unsigned int)((U1 / Uunits) * pwmMax);
  } else {
    digitalWrite(IN_1_1, LOW);  digitalWrite(IN_2_1, HIGH);
    dir_cmd_1 = -1;
    float U1 = clampf(-u_n_1, 0.0f, (float)Uunits);
    pwm_control_1 = (unsigned int)((U1 / Uunits) * pwmMax);
  }
  unsigned int pwmDuty_1 = pwm_control_1 + PWM_MIN_MOTOR_1;
  if (pwmDuty_1 > pwmMax) pwmDuty_1 = pwmMax;
  analogWriteADJ(OutputPWM_GPIO_1, pwmDuty_1);

  // --- Motor 2 ---
  unsigned int pwm_control_2 = 0;
  if (u_n_2 >= 0){
    digitalWrite(IN_1_2, HIGH); digitalWrite(IN_2_2, LOW);
    dir_cmd_2 = +1;
    float U2 = clampf(u_n_2, 0.0f, (float)Uunits);
    pwm_control_2 = (unsigned int)((U2 / Uunits) * pwmMax);
  } else {
    digitalWrite(IN_1_2, LOW);  digitalWrite(IN_2_2, HIGH);
    dir_cmd_2 = -1;
    float U2 = clampf(-u_n_2, 0.0f, (float)Uunits);
    pwm_control_2 = (unsigned int)((U2 / Uunits) * pwmMax);
  }
  unsigned int pwmDuty_2 = pwm_control_2 + PWM_MIN_MOTOR_2;
  if (pwmDuty_2 > pwmMax) pwmDuty_2 = pwmMax;
  analogWriteADJ(OutputPWM_GPIO_2, pwmDuty_2);

  // --- Telemetría compacta (decimada) ---
#if ENABLE_TELEMETRY
  if (++tel_cnt >= TEL_DIV) {
    tel_cnt = 0;
    if (Serial && Serial.availableForWrite() >= 32) {
          Serial.print("Angulo 1: ");
      Serial.print(angulo_1,1);  Serial.print(',');
      Serial.print("Angulo 2: ");
      Serial.print(angulo_2,1);  Serial.print(',');
      Serial.print("Referencia 1: ");
      Serial.print(Ref_1_deg,1); Serial.print(',');
      Serial.print("Referencia 2: ");
      Serial.print(Ref_2_deg,1); Serial.print(',');
      Serial.print("Error 1: ");
      Serial.print(e_n_1,1);     Serial.print(',');
      Serial.print("Error 2: ");
      Serial.print(e_n_2,1);     Serial.print(',');
      Serial.print("PWM 1: ");
      Serial.print(pwmDuty_1);   Serial.print(',');
      Serial.print("PWM 2: ");
      Serial.println(pwmDuty_2);
    }

  }
#endif
}

void setup(){
#if ENABLE_TELEMETRY
  Serial.begin(115200);
#endif

  pinMode(Sensor_1, INPUT_PULLUP);
  pinMode(Sensor_2, INPUT_PULLUP);
  pinMode(IN_1_1, OUTPUT); pinMode(IN_2_1, OUTPUT);
  pinMode(IN_1_2, OUTPUT); pinMode(IN_2_2, OUTPUT);

  // Interrupciones encoders (flanco configurable)
  attachInterrupt(digitalPinToInterrupt(Sensor_1), isrSlot1, COUNT_ON_FALL_1 ? FALLING : RISING);
  attachInterrupt(digitalPinToInterrupt(Sensor_2), isrSlot2, COUNT_ON_FALL_2 ? FALLING : RISING);

  // PWM
  setupPWMadj();
  analogWriteADJ(OutputPWM_GPIO_1, 0);
  analogWriteADJ(OutputPWM_GPIO_2, 0);

  // Inicialización de contadores
  noInterrupts();
    pulseCount_1 = pulseCount_2 = 0;
    prevPulse_1  = prevPulse_2  = 0;
    signedCount_1 = signedCount_2 = 0;
  interrupts();

  // Sentido inicial (por si al arranque mandas muy poco PWM)
  dir_cmd_1 = +1;
  dir_cmd_2 = +1;
}

void loop(){
  calibracion();
}
