#include <Arduino.h>

// --- تعريف البنات ---
#define ENCODER_A 2
#define ENCODER_B 3
#define ENA 6
#define IN1 8
#define IN2 9
#define POT_PIN A0
#define ACS_PIN A1

// --- إعدادات الحساس والطاقة ---
const float SENSITIVITY = 0.185;   // 5A = 0.185 | 20A = 0.100 | 30A = 0.066
const float SUPPLY_VOLTAGE = 12.0;
float zeroVoltageOffset = 2.5;     // بتتعاير تلقائياً في setup

// --- متغيرات القياس ---
float motorCurrent = 0.0;
float motorVoltage = 0.0;
int currentPWM = 0;

// --- متغيرات الإنكودر ---
volatile long encoderTicks = 0;
const float TICKS_PER_REV = 950.0;

// --- متغيرات السرعة ---
long lastTicksForRPM = 0;
unsigned long lastTimeForRPM = 0;
float currentRPM = 0.0;
float targetRPM = 0.0;

// --- معاملات الـ PID للزاوية (Position PID) ---
float kp_pos = 1.2, ki_pos = 0.05, kd_pos = 0.2;
float targetAngle = 0.0;
float posIntegral = 0.0, lastPosError = 0.0;
unsigned long lastPosTime = 0;
bool lastUseAngleMode = false;

// --- معاملات الـ PID للسرعة (Speed PID) ---
float kp_speed = 2.0, ki_speed = 5.0;
float speedIntegral = 0.0;

unsigned long lastSendTime = 0;

// --- متغيرات وضع التحكم والتواصل مع يونيتي ---
bool autoMode = false;
int manualDir = 0;
float manualSpeed = 30.0;

char rxBuffer[32];
byte rxIndex = 0;

void driveMotorRaw(int pwmSpeed);
void driveMotorPositionPID(float target, float current, float dt);
void calibrateCurrentSensor();
float floatMap(float x, float in_min, float in_max, float out_min, float out_max);

// --- دالة تفسير الأوامر ---
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      rxBuffer[rxIndex] = '\0';

      if (strcmp(rxBuffer, "CW") == 0)               manualDir = 1;
      else if (strcmp(rxBuffer, "CCW") == 0)         manualDir = -1;
      else if (strcmp(rxBuffer, "STOP") == 0)        manualDir = 0;
      else if (strcmp(rxBuffer, "MODE_AUTO") == 0)   autoMode = true;
      else if (strcmp(rxBuffer, "MODE_MANUAL") == 0) autoMode = false;
      else if (strncmp(rxBuffer, "SPD:", 4) == 0) {
        manualSpeed = constrain(atof(rxBuffer + 4), 0.0, 60.0);
      }
      else if (strncmp(rxBuffer, "ANG:", 4) == 0) {
        targetAngle = atof(rxBuffer + 4);
        if (autoMode) manualDir = 0;
      }

      rxIndex = 0;
    } else if (c != '\r' && rxIndex < 31) {
      rxBuffer[rxIndex++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // إيقاف الموتور تماماً قبل معايرة حساس التيار
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  delay(100);

  calibrateCurrentSensor();

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, RISING);

  lastPosTime = millis();
  lastTimeForRPM = millis();
}

void loop() {
  handleSerial();

  int potValue = analogRead(POT_PIN);
  unsigned long currentTime = millis();

  long currentTicks;
  noInterrupts();
  currentTicks = encoderTicks;
  interrupts();
  float currentAngle = (currentTicks / TICKS_PER_REV) * 360.0;

  bool potAngleMode = (potValue > 340 && potValue < 681);
  bool useAngleMode = autoMode ? (manualDir == 0) : potAngleMode;

  // تصفير قفزة الـ Derivative عند الدخول لوضع الزاوية
  if (useAngleMode && !lastUseAngleMode) {
    lastPosError = targetAngle - currentAngle;
    posIntegral = 0;
  }
  lastUseAngleMode = useAngleMode;

  if (currentTime - lastTimeForRPM >= 50) {
    long deltaTicks = currentTicks - lastTicksForRPM;
    float dtRPM = (currentTime - lastTimeForRPM) / 1000.0;

    currentRPM = ((float)deltaTicks / TICKS_PER_REV) * (60.0 / dtRPM);
    lastTicksForRPM = currentTicks;
    lastTimeForRPM = currentTime;

    if (!useAngleMode) {
      if (autoMode) {
        targetRPM = manualDir * manualSpeed;
      } else {
        if (potValue <= 340) targetRPM = floatMap(potValue, 0, 340, -60.0, -10.0);
        else if (potValue >= 681) targetRPM = floatMap(potValue, 681, 1023, 10.0, 60.0);
      }

      float speedError = targetRPM - currentRPM;
      speedIntegral += speedError * dtRPM;

      if (speedIntegral > 100) speedIntegral = 100;
      if (speedIntegral < -100) speedIntegral = -100;

      float feedForward = 0;
      if (targetRPM > 0) feedForward = 65.0 + (abs(targetRPM) / 60.0) * (255.0 - 65.0);
      else if (targetRPM < 0) feedForward = -(65.0 + (abs(targetRPM) / 60.0) * (255.0 - 65.0));

      float totalPWM = feedForward + (kp_speed * speedError) + (ki_speed * speedIntegral);
      driveMotorRaw((int)totalPWM);

      targetAngle = currentAngle;
      posIntegral = 0;
      lastPosError = 0;
    }
  }

  if (useAngleMode) {
    targetRPM = 0.0;
    speedIntegral = 0.0;

    float dtPos = (currentTime - lastPosTime) / 1000.0;
    if (dtPos <= 0) dtPos = 0.001;

    driveMotorPositionPID(targetAngle, currentAngle, dtPos);
    lastPosTime = currentTime;
  } else {
    lastPosTime = currentTime;
  }

  // قراءة التيار وإرسال 7 قيم ليونيتي كل 50ms
  if (currentTime - lastSendTime > 50) {
    long sum = 0;
    for (int i = 0; i < 50; i++) sum += analogRead(ACS_PIN);   // 50 قراءة بدل 20
    float avgRaw = sum / 50.0;
    float v_sensor = avgRaw * (5.0 / 1024.0);

    motorCurrent = (v_sensor - zeroVoltageOffset) / SENSITIVITY;
    if (abs(motorCurrent) < 0.02) motorCurrent = 0.0;          // dead zone أصغر

    motorVoltage = (abs(currentPWM) / 255.0) * SUPPLY_VOLTAGE;

    Serial.print(targetAngle); Serial.print(",");
    Serial.print(currentAngle); Serial.print(",");
    Serial.print(targetRPM); Serial.print(",");
    Serial.print(currentRPM); Serial.print(",");
    Serial.print(motorCurrent); Serial.print(",");
    Serial.print(motorVoltage); Serial.print(",");
    Serial.println(autoMode ? 1 : 0);

    lastSendTime = currentTime;
  }
}

void readEncoder() {
  int b = digitalRead(ENCODER_B);
  if (b > 0) encoderTicks++;
  else encoderTicks--;
}

void driveMotorRaw(int pwmSpeed) {
  if (pwmSpeed > 255) pwmSpeed = 255;
  if (pwmSpeed < -255) pwmSpeed = -255;

  currentPWM = pwmSpeed;

  if (pwmSpeed > 5) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmSpeed);
  } else if (pwmSpeed < -5) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwmSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void driveMotorPositionPID(float target, float current, float dt) {
  float error = target - current;
  posIntegral += error * dt;
  float derivative = (error - lastPosError) / dt;
  float power = (kp_pos * error) + (ki_pos * posIntegral) + (kd_pos * derivative);
  lastPosError = error;

  int pwmVal = (int)abs(power);
  if (pwmVal > 255) pwmVal = 255;

  if (abs(error) < 1.0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    currentPWM = 0;
    return;
  }

  if (pwmVal < 65) pwmVal = 65;

  if (power > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    currentPWM = pwmVal;
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    currentPWM = -pwmVal;
  }
  analogWrite(ENA, pwmVal);
}

void calibrateCurrentSensor() {
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(ACS_PIN);
    delay(2);
  }
  float avgRaw = sum / 100.0;
  zeroVoltageOffset = avgRaw * (5.0 / 1024.0);
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}