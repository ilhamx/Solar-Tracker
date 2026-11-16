/*********** BLYNK ***********/

#include <Arduino.h>
#include "Adafruit_INA3221.h"

// Create an INA3221 object
Adafruit_INA3221 ina;

// Blynk setup
#define BLYNK_TEMPLATE_ID "TMPL6eR1bVog2"
#define BLYNK_TEMPLATE_NAME "Solar Tracker"
#define BLYNK_AUTH_TOKEN "XOPuG9s9pjBzVcIWq-BsI5d6jrSCh5Cq"

#define BLYNK_PRINT Serial

// #define Wire I2CBus1
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "noni";
char pass[] = "1234567890";

BlynkTimer timer; // Create a timer object to handle timed events

/*********** I2C ***********/
#include <Wire.h>
// TwoWire I2CBus1 = TwoWire(0); // default
TwoWire I2CBus2 = TwoWire(1); // second bus
SemaphoreHandle_t i2cMutex;

float yaw0 = 0;
float currentYaw = 0;
float L1, L2, L3, L4;                             // light levels from 4 sensors
float L1_kalman, L2_kalman, L3_kalman, L4_kalman; // kalman-filtered light levels
float sum, nx, ny;                                // for fusion calculations

#define YAW_LIMIT 160 // max rotation from start (degrees)

/*********** BH1750 ***********/
#include <BH1750.h>

BH1750 bh1; // Bus1 addr 0x23
BH1750 bh2; // Bus1 addr 0x5C
BH1750 bh3; // Bus2 addr 0x23
BH1750 bh4; // Bus2 addr 0x5C

/*********** SERVO ***********/
#include <ESP32Servo.h>
Servo servoPan;
Servo servoTilt;

#define PAN_PIN 17
#define TILT_PIN 16

float panAngle = 1500;
float tiltAngle = 1500;

// Manual control flag: when true, automatic tracking (updateSens) is paused
volatile bool manualMode = false;

// Helper functions untuk konversi dan clamp
int clampInt(int v, int lo, int hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

// Konversi derajat (0-180) ke microseconds antara minUs..maxUs
int degToUs(int deg, int minUs, int maxUs)
{
  deg = clampInt(deg, 0, 180);
  return map(deg, 0, 180, minUs, maxUs);
}

// Konversi microseconds ke derajat
int usToDeg(int us, int minUs, int maxUs)
{
  us = clampInt(us, minUs, maxUs);
  return map(us, minUs, maxUs, 0, 180);
}

#define PAN_MIN 500
#define PAN_MAX 2500
#define TILT_MIN 900
#define TILT_MAX 1700

// Skala derajat fisik untuk tilt (sesuai kemiringan aktual panel), dipakai
// untuk command TILT/TILT_DELTA/STATUS supaya konsisten dengan log "Kemiringan"
#define TILT_DEG_MIN 45
#define TILT_DEG_MAX 135

int tiltDegToUs(int deg)
{
  deg = clampInt(deg, TILT_DEG_MIN, TILT_DEG_MAX);
  return map(deg, TILT_DEG_MIN, TILT_DEG_MAX, TILT_MIN, TILT_MAX);
}

int tiltUsToDeg(int us)
{
  us = clampInt(us, TILT_MIN, TILT_MAX);
  return map(us, TILT_MIN, TILT_MAX, TILT_DEG_MIN, TILT_DEG_MAX);
}

// global variables for ina3221
float shuntV0, busV0, current0;
float shuntV1, busV1, current1;
float shuntV2, busV2, current2;

/*********** TRACKER PARAM ***********/
float Kp_X; // pan is more sensitive to error, so we use a larger Kp
float Kp_Y; // tilt is less sensitive to error, so we use a smaller Kp
float deadzone = 0.08;
unsigned long lastUpdate = 0;

float kalmanUpdate(float measurement)
{

  float x = 0;    // estimate
  float P = 1;    // estimation error
  float Q = 0.01; // process noise
  float R = 1;    // measurement noise

  // Predict
  P = P + Q;

  // Kalman Gain
  float K = P / (P + R);

  // Update estimate
  x = x + K * (measurement - x);

  // Update error covariance
  P = (1 - K) * P;

  return x;
}

int currentPan = 600;   // posisi awal servo (microseconds)
int currentTilt = 1500; // posisi awal servo (microseconds)
int stepSize = 1;       // kecepatan gerak (semakin kecil = semakin halus)
int deadband = 10;      // toleransi biar gak jitter

void writePan(int targetUs)
{
  int diff = targetUs - currentPan;

  // Serial.printf("Moving servo to %d us (target: %d us)\n", currentPan, targetUs);
  // Jika perubahan kecil, abaikan (anti jitter)
  // if (abs(diff) < deadband) {
  //   return;
  // }

  // Gerakan bertahap
  if (diff > 0)
  {
    currentPan += stepSize;
  }
  else
  {
    currentPan -= stepSize;
  }

  servoPan.writeMicroseconds(currentPan);
}

// Fungsi khusus baca sensor cahaya saja (tanpa kontrol servo)
// Dipakai saat manualMode aktif, supaya L1-L4 tetap ter-update
// walaupun tracking otomatis (updateSens) sedang di-skip.
void readSensors()
{
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
  {
    L4 = bh4.readLightLevel();
    L2 = bh2.readLightLevel();
    xSemaphoreGive(i2cMutex);
  }
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
  {
    L3 = bh3.readLightLevel();
    L1 = bh1.readLightLevel();
    xSemaphoreGive(i2cMutex);
  }

  L1 = kalmanUpdate(L1);
  L2 = kalmanUpdate(L2);
  L3 = kalmanUpdate(L3);
  L4 = kalmanUpdate(L4);

  sum = L1 + L2 + L3 + L4 + 0.001;
}

void updateSens()
{
  /*********** READ SENSORS ***********/
  // Serial.println("reading sensor");

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
  {
    // 🔒 LOCK I2C
    L4 = bh4.readLightLevel(); // top-left
    L2 = bh2.readLightLevel(); // top-right
    xSemaphoreGive(i2cMutex);  // 🔓 UNLOCK
  }
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
  {
    // 🔒 LOCK I2C
    L3 = bh3.readLightLevel(); // bottom-left
    L1 = bh1.readLightLevel(); // bottom-right
    xSemaphoreGive(i2cMutex);  // 🔓 UNLOCK
  }

  L1 = kalmanUpdate(L1); // example of using Kalman filter on one sensor (can be applied to all);
  L2 = kalmanUpdate(L2); // example of using Kalman filter on one sensor (can be applied to all);
  L3 = kalmanUpdate(L3); // example of using Kalman filter on one sensor (can be applied to all);
  L4 = kalmanUpdate(L4); // example of using Kalman filter on one sensor (can be applied to all);

  sum = L1 + L2 + L3 + L4 + 0.001;

  if (sum < 50) // night protection
  {
    return;
  }

  /*********** FUSION **********
   * Calculate normalized error in X and Y directions based on light levels
   * The idea is to find the imbalance in light intensity across the sensors
   */
  float errX = (L1 + L2) - (L3 + L4);
  float errY = (L1 + L3) - (L2 + L4);

  if (tiltAngle > 1500)
    nx = errX / sum;
  else
    nx = -errX / sum;
  ny = errY / sum;

  if (abs(nx) < deadzone)
    nx = 0;
  if (abs(ny) < deadzone)
    ny = 0;

  /*********** CONTROL ***********/
  Kp_X = 7; // adjust as needed
  Kp_Y = 5; // adjust as needed

  panAngle += nx * Kp_X;
  tiltAngle += ny * Kp_Y;

  panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
  tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);

  static int panMinCounter = 0;

  if (panAngle <= PAN_MIN)
  {
    panMinCounter++;
    if (panMinCounter >= 150)
    {
      int us = 500;
      while (1)
      {
        us += 10;
        // servoTilt.write(90); // keep tilt at 900 us while pan is recovering
        servoPan.writeMicroseconds(us);
        if (us >= 1500)
        {
          break; // stop after reaching 1500 us
        }
        vTaskDelay(pdMS_TO_TICKS(70)); // Delay 100ms
      }
      panAngle = 1500;
      tiltAngle = 1500;
      return;
    }
  }
  else
  {
    panMinCounter = 0;
  }

  static int panMaxCounter = 0;

  if (panAngle >= PAN_MAX)
  {
    panMaxCounter++;
    if (panMaxCounter >= 150)
    {
      int us = 2400;
      while (1)
      {
        us -= 10;
        // servoTilt.write(90); // keep tilt at 900 us while pan is recovering
        servoPan.writeMicroseconds(us);
        if (us <= 1500)
        {
          break; // stop after reaching 1500 us
        }
        vTaskDelay(pdMS_TO_TICKS(70)); // Delay 100ms
      }
      panAngle = 1500;
      tiltAngle = 1500;
      return;
    }
  }
  else
  {
    panMaxCounter = 0;
  }

  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);
}

void readINA()
{

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
  {
    // 🔒 LOCK I2C
    // shuntV0 = ina.getShuntVoltage(0);
    busV0 = ina.getBusVoltage(0);
    current0 = -ina.getCurrentAmps(0);

    // shuntV1 = ina.getShuntVoltage(1);
    busV1 = ina.getBusVoltage(1);
    current1 = -ina.getCurrentAmps(1);

    // shuntV2 = ina.getShuntVoltage(2);
    busV2 = ina.getBusVoltage(2);
    current2 = ina.getCurrentAmps(2);

    xSemaphoreGive(i2cMutex); // 🔓 UNLOCK
  }

  if (isnan(shuntV0) || isnan(busV0) || isnan(current0) ||
      isnan(shuntV1) || isnan(busV1) || isnan(current1) ||
      isnan(shuntV2) || isnan(busV2) || isnan(current2))
  {
    BLYNK_LOG("Failed to read INA3221");
    return;
  }
  else
  {
    // BLYNK_LOG("Light Levels: L1=%.0f, L2=%.0f, L3=%.0f, L4=%.0f", L1, L2, L3, L4);
  }
}

void checkWifi(){
  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(2, LOW);
  }
  else
  {
    digitalWrite(2, HIGH);
  }
}

void sendBlynk() // send INA3221 data to Blynk every 3 seconds
{
  if (isnan(busV0) || isnan(current0) ||
      isnan(busV1) || isnan(current1) ||
      isnan(busV2) || isnan(current2))
  {
    BLYNK_LOG("Cannot send INA3221 data to Blynk due to NaN values");
    return;
  }

  int panDeg = map(panAngle, PAN_MIN, PAN_MAX, 0, 180);
  int tiltDeg = servoTilt.read();
  // int tiltDeg = map(tiltAngle, TILT_MIN, TILT_MAX, 0, 180);

  Blynk.virtualWrite(V0, panDeg);
  Blynk.virtualWrite(V1, tiltDeg);
  Blynk.virtualWrite(V2, current0);
  Blynk.virtualWrite(V3, busV0);
  Blynk.virtualWrite(V4, current1);
  Blynk.virtualWrite(V5, busV1);
  Blynk.virtualWrite(V6, current2); 
  Blynk.virtualWrite(V7, busV2);
  Blynk.virtualWrite(V8,  L1);
  Blynk.virtualWrite(V9,  L2);
  Blynk.virtualWrite(V10, L3);
  Blynk.virtualWrite(V11, L4);
}

// Format perintah serial (case-insensitive, newline-terminated):
// MANUAL ON            -> aktifkan mode manual (hentikan tracking otomatis)
// MANUAL OFF           -> kembali ke mode otomatis
// PAN <deg>            -> set pan ke sudut derajat (0-180)
// TILT <deg>           -> set tilt ke sudut derajat (0-180)
// PAN_US <us>          -> set pan microseconds langsung (min..max)
// TILT_US <us>         -> set tilt microseconds langsung (min..max)
// PAN_DELTA <deg>      -> ubah pan relatif dalam derajat (bisa negatif)
// TILT_DELTA <deg>     -> ubah tilt relatif dalam derajat (bisa negatif)
// READINA              -> baca data dari INA3221
// READSENS             -> baca data dari sensor cahaya
// STEP <n>             -> set stepSize (microseconds per update)
// HOME                 -> kembalikan ke tengah
// STATUS               -> print status saat ini
// Contoh: "PAN 90" atau "PAN_DELTA -5"

void ReadString(void *pvParameters)
{
  String buf;
  for (;;)
  {
    while (Serial.available())
    {
      char c = Serial.read();
      if (c == '\r')
        continue;
      if (c == '\n')
      {
        buf.trim();
        if (buf.length() > 0)
        {
          // parse
          String cmd = buf;
          cmd.toUpperCase();

          // Tokenize
          int sp = cmd.indexOf(' ');
          String verb = (sp == -1) ? cmd : cmd.substring(0, sp);
          String arg = (sp == -1) ? "" : cmd.substring(sp + 1);

          if (verb == "MANUAL")
          {
            arg.trim();
            if (arg == "ON")
            {
              manualMode = true;
              Serial.println("Manual mode: ON");
            }
            else if (arg == "OFF")
            {
              manualMode = false;
              Serial.println("Manual mode: OFF");
            }
            else
            {
              Serial.println("Usage: MANUAL ON|OFF");
            }
          }
          else if (verb == "PAN" || verb == "TILT")
          {
            if (arg.length() == 0)
            {
              Serial.println("Provide degree value, e.g. PAN 90");
            }
            else
            {
              int deg = arg.toInt();
              if (verb == "PAN")
              {
                int us = degToUs(deg, PAN_MIN, PAN_MAX);
                panAngle = us;
                servoPan.writeMicroseconds(clampInt(us, PAN_MIN, PAN_MAX));
                Serial.printf("PAN set: %d deg -> %d us\n", deg, us);
              }
              else
              {
                int us = tiltDegToUs(deg);
                tiltAngle = us;
                servoTilt.writeMicroseconds(clampInt(us, TILT_MIN, TILT_MAX));
                Serial.printf("TILT set: %d deg -> %d us\n", deg, us);
              }
            }
          }
          else if (verb == "PAN_US" || verb == "TILT_US")
          {
            if (arg.length() == 0)
            {
              Serial.println("Provide microseconds value, e.g. PAN_US 1500");
            }
            else
            {
              int us = arg.toInt();
              if (verb == "PAN_US")
              {
                us = clampInt(us, PAN_MIN, PAN_MAX);
                panAngle = us;
                servoPan.writeMicroseconds(us);
                Serial.printf("PAN_US set: %d us\n", us);
              }
              else
              {
                us = clampInt(us, TILT_MIN, TILT_MAX);
                tiltAngle = us;
                servoTilt.writeMicroseconds(us);
                Serial.printf("TILT_US set: %d us\n", us);
              }
            }
          }
          else if (verb == "PAN_DELTA" || verb == "TILT_DELTA")
          {
            if (arg.length() == 0)
            {
              Serial.println("Provide delta degrees, e.g. PAN_DELTA -5");
            }
            else
            {
              int d = arg.toInt();
              if (verb == "PAN_DELTA")
              {
                int curDeg = usToDeg((int)panAngle, PAN_MIN, PAN_MAX);
                int nd = clampInt(curDeg + d, 0, 180);
                int us = degToUs(nd, PAN_MIN, PAN_MAX);
                panAngle = us;
                servoPan.writeMicroseconds(us);
                Serial.printf("PAN_DELTA: now %d deg -> %d us\n", nd, us);
              }
              else
              {
                int curDeg = tiltUsToDeg((int)tiltAngle);
                int nd = clampInt(curDeg + d, TILT_DEG_MIN, TILT_DEG_MAX);
                int us = tiltDegToUs(nd);
                tiltAngle = us;
                servoTilt.writeMicroseconds(us);
                Serial.printf("TILT_DELTA: now %d deg -> %d us\n", nd, us);
              }
            }
          }
          else if (verb == "STEP")
          {
            if (arg.length() == 0)
            {
              Serial.printf("Current stepSize: %d\n", stepSize);
            }
            else
            {
              int s = arg.toInt();
              if (s <= 0)
                Serial.println("Step must be > 0");
              else
              {
                stepSize = s;
                Serial.printf("stepSize set to %d\n", stepSize);
              }
            }
          }
          else if (verb == "HOME")
          {
            panAngle = (PAN_MIN + PAN_MAX) / 2;    // 1500 us
            tiltAngle = (TILT_MIN + TILT_MAX) / 2; // 1300 us
            servoPan.writeMicroseconds((int)panAngle);
            servoTilt.writeMicroseconds((int)tiltAngle);
            Serial.printf("Homing: pan -> %d us | tilt -> %d us\n", (int)panAngle, (int)tiltAngle);
          }
          else if (verb == "STATUS")
          {
            int panDeg = usToDeg((int)panAngle, PAN_MIN, PAN_MAX);
            int tiltDeg = tiltUsToDeg((int)tiltAngle);
            Serial.printf("Manual:%s | pan=%d us (%d deg) | tilt=%d us (%d deg) | step=%d\n", manualMode ? "ON" : "OFF", (int)panAngle, panDeg, (int)tiltAngle, tiltDeg, stepSize);
          }
          else if (verb == "READINA")
          {
            readINA();
            Serial.printf("CH1: Bus %.3f V | Current %.3f A\n", busV0, current0);
            Serial.printf("CH2: Bus %.3f V | Current %.3f A\n", busV1, current1);
            Serial.printf("CH3: Bus %.3f V | Current %.3f A\n", busV2, current2);
          }
          else if (verb == "READSENS")
          {
            if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
            {
              // 🔒 LOCK I2C
              L1 = bh1.readLightLevel(); // top-left
              L2 = bh2.readLightLevel(); // top-right
              L3 = bh3.readLightLevel(); // bottom-left
              L4 = bh4.readLightLevel(); // bottom-right
              xSemaphoreGive(i2cMutex);  // 🔓 UNLOCK
            }
            sum = L1 + L2 + L3 + L4;
            Serial.printf("L1=%.0f | L2=%.0f | L3=%.0f | L4=%.0f | Sum=%.0f\n", L1, L2, L3, L4, sum);
          }
          else
          {
            Serial.println("Unknown command. Use MANUAL, PAN, TILT, PAN_DELTA, TILT_DELTA, PAN_US, TILT_US, STEP, HOME, STATUS, READINA, READSENS");
          }
        }
        buf = "";
      }
      else
      {
        buf += c;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void TaskCalc(void *pvParameters)
{
  for (;;)
  {

    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1)
    {
      // kerja kamu di sini
      // Jika manualMode aktif, kita skip updateSens (tracking otomatis dimatikan)
      // tapi tetap panggil readSensors() supaya L1-L4 tetap ter-update
      // dan datanya tetap fresh saat dikirim ke Blynk / dicetak ke Serial.
      if (!manualMode)
      {
        updateSens();
      }
      else
      {
        readSensors();
      }
      vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20));
    }
  }
}

void TaskBlynk(void *pvParameters)
{
  for (;;)
  {
    Blynk.run();
    timer.run();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void TaskUpdateSensors(void *pvParameters)
{
  for (;;)
  {
    // Print status ke Serial Monitor selalu berjalan, baik Manual ON maupun OFF
    Serial.printf("CH1: Bus %.3f V | Current %.3f A\n", busV0, current0);
    Serial.printf("CH2: Bus %.3f V | Current %.3f A\n", busV1, current1);
    Serial.printf("CH3: Bus %.3f V | Current %.3f A\n", busV2, current2);
    float panDeg = map(panAngle, PAN_MIN, PAN_MAX, 0, 180);
    float tiltDeg = map(tiltAngle, TILT_MIN, TILT_MAX, TILT_DEG_MIN, TILT_DEG_MAX);
    Serial.printf("L1=%.0f | L2=%.0f | L3=%.0f | L4=%.0f | Sum=%.0f | nx=%.3f ny=%.3f | Pan=%.1f Tilt=%.1f | Rotasi=%.1f | Kemiringan=%.1f | Manual=%s\n", L1, L2, L3, L4, sum, nx, ny, panAngle, tiltAngle, panDeg, tiltDeg, manualMode ? "ON" : "OFF");

    // Serial.printf("L1:%.0f L2:%.0f L3:%.0f L4:%.0f ", L1, L2, L3, L4);                                                                  // for plotting kalman effect in serial plotter
    // Serial.printf("L1_filtered:%.0f L2_filtered:%.0f L3_filtered:%.0f L4_filtered:%.0f\n", L1_kalman, L2_kalman, L3_kalman, L4_kalman); // for plotting kalman effect in serial plotter

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  /*********** INIT I2C ***********/
  Serial.println("Initializing I2C buses...");
  Wire.begin(21, 22, 100000); // SDA, SCL
  I2CBus2.begin(19, 18, 100000);

  if (!ina.begin(0x40, &Wire))
  { // can use other I2C addresses or buses
    Serial.println("Failed to find INA3221 chip");
    while (1)
      delay(10);
  }
  Serial.println("INA3221 Found!");

  ina.setAveragingMode(INA3221_AVG_16_SAMPLES);
  for (uint8_t i = 0; i < 3; i++)
  {
    ina.setShuntResistance(i, 0.1); // set shunt resistor value in ohms (adjust as needed)
  }

  /*********** INIT BH1750 ***********/
  Serial.println("Initializing BH1750 sensors...");
  if (!bh4.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire))
    Serial.println("bh4 (Bus1, 0x23) FAILED to init!");
  else
    Serial.println("bh4 (Bus1, 0x23) OK");

  if (!bh2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire))
    Serial.println("bh2 (Bus1, 0x5C) FAILED to init!");
  else
    Serial.println("bh2 (Bus1, 0x5C) OK");

  if (!bh3.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2CBus2))
    Serial.println("bh3 (Bus2, 0x23) FAILED to init!");
  else
    Serial.println("bh3 (Bus2, 0x23) OK");

  if (!bh1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &I2CBus2))
    Serial.println("bh1 (Bus2, 0x5C) FAILED to init!");
  else
    Serial.println("bh1 (Bus2, 0x5C) OK");

  /*********** SERVO ***********/
  Serial.println("Initializing servos...");
  servoPan.setPeriodHertz(200); // standard 200 Hz for servos
  servoTilt.setPeriodHertz(200);
  servoPan.attach(PAN_PIN);
  servoTilt.attach(TILT_PIN);
  servoPan.writeMicroseconds(panAngle);
  servoTilt.writeMicroseconds(tiltAngle);

  i2cMutex = xSemaphoreCreateMutex();
  
  if (i2cMutex == NULL)
  {
    Serial.println("Mutex gagal dibuat!");
  }

  xTaskCreatePinnedToCore(TaskBlynk, "blynk", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskUpdateSensors, "sensors", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskCalc, "TaskCalc", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ReadString, "readString", 4096, NULL, 1, NULL, 0);

  /*********** BLYNK ***********/
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass); // Manually connect to Wi-Fi
  Blynk.config(BLYNK_AUTH_TOKEN); // Configure Blynk, but don't connect yet

  timer.setInterval(500L, readINA);     // read INA3221 every n ms
  timer.setInterval(30000L, sendBlynk); // send data to Blynk (adjust as needed)
  timer.setInterval(1000L, checkWifi); // send data to Blynk (adjust as needed)

}

void loop()
{
}