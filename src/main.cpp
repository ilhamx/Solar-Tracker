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
// Fungsi helper terpusat untuk mengeksekusi perintah (baik dari Serial maupun MQTT)

#include <Arduino.h>
SemaphoreHandle_t i2cMutex;
#include <oled.h>
#include <DallasTemperature.h>

// Konfigurasi pin dll...
OneWire oneWire(23);
DallasTemperature sensors(&oneWire);

// Create an INA3221 object
#include "INA226.h"
INA226 INA(0x40);

// #define Wire I2CBus1
#include <WiFi.h>
#include "mqtt_client.h"

char ssid[] = "LPG_3KG";
char pass[] = "piulbaikhati";

/*********** I2C ***********/
#include <Wire.h>

// Pin Sensor LDR 1-4
#define LDR1 35
#define LDR2 34
#define LDR3 36
#define LDR4 39

// batas hasil pengukuran nilai maksimum dan minimum ldr, untuk menentukan apakah sensor terjadi masalah.
#define MAX_LDR 5
#define MIN_LDR 1018

#define LED_WIFI 2
#define LED_STATUS 15

/*********** TRACKER PARAM ***********/

// Queue handle untuk menampung string log (kapasitas 10 antrian)
QueueHandle_t serialLogQueue;
float Kp_X; // pan is more sensitive to error, so we use a larger Kp
float Kp_Y; // tilt is less sensitive to error, so we use a smaller Kp
float deadzone = 0.08;
unsigned long lastUpdate = 0;

int stepSize = 1;  // kecepatan gerak (semakin kecil = semakin halus)
int deadband = 10; // toleransi biar gak jitter

bool ledStatus = false, isWifiConnected = false;
float yaw0 = 0;
float currentYaw = 0;
int currentPan = 600;   // posisi awal servo (microseconds)
int currentTilt = 1500; // posisi awal servo (microseconds)
float busVoltage, shuntVoltage, current, power;
float L1, L2, L3, L4, tempC;                      // light levels from 4 sensors
float sum, nx, ny;                                // for fusion calculations
float L1_kalman, L2_kalman, L3_kalman, L4_kalman; // kalman-filtered light levels

#define YAW_LIMIT 160 // max rotation from start (degrees)

/*********** SERVO ***********/
#include <ESP32Servo.h>
Servo servoPan;
Servo servoTilt;

#define PAN_PIN 17
#define TILT_PIN 16

#define PAN_MIN 500
#define PAN_MAX 2500
#define TILT_MIN 900
#define TILT_MAX 1700

// Skala derajat fisik untuk tilt (sesuai kemiringan aktual panel), dipakai
// untuk command TILT/TILT_DELTA/STATUS supaya konsisten dengan log "Kemiringan"
#define TILT_DEG_MIN 45
#define TILT_DEG_MAX 135

float panAngle = 1500;
float tiltAngle = 1500;

// Manual control flag: when true, automatic tracking (updateSens) is paused
volatile bool manualMode = false;

// Konfigurasi Broker MQTT WSS
const char *mqtt_broker_uri = "wss://mqtt.solar2axis.my.id:443/mqtt";
const char *mqtt_broker_uri_local = "mqtt://192.168.4.2:1883";
const char *topic_pub_ina = "tracker/sensor/ina226";
const char *topic_pub_ldr = "tracker/sensor/ldr";
const char *topic_pub_servo = "tracker/servo/status";
const char *topic_pub_temp = "tracker/sensor/temperature";
const char *topic_pub_log = "tracker/log/serial";
const char *topic_sub = "tracker/perintah";

esp_mqtt_client_handle_t client;
bool isMqttConnected = false;
unsigned long lastPublish = 0;

void executeCommand(String cmd);
// Handler untuk menangkap semua event MQTT (Koneksi, Data Masuk, Error)
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

  switch ((esp_mqtt_event_id_t)event_id)
  {
  case MQTT_EVENT_CONNECTED:
    Serial.println("[MQTT] Terhubung ke broker WSS!");
    isMqttConnected = true;
    // Subscribe ke topik setelah berhasil terhubung
    esp_mqtt_client_subscribe(client, topic_sub, 1);
    Serial.printf("[MQTT] Subscribed ke topik: %s\n", topic_sub);
    break;

  case MQTT_EVENT_DISCONNECTED:
    Serial.println("[MQTT] Terputus dari broker.");
    isMqttConnected = false;
    break;

  case MQTT_EVENT_DATA:
  {
    Serial.println("[MQTT] Pesan diterima!");
    Serial.printf("Topik : %.*s\n", event->topic_len, event->topic);
    Serial.printf("Data  : %.*s\n", event->data_len, event->data);

    // Ekstrak payload string dari event data
    String payload = String(event->data).substring(0, event->data_len);
    payload.trim();

    if (payload.length() > 0)
    {
      // Jalankan perintah menggunakan fungsi helper yang sama dengan Serial
      executeCommand(payload);
    }
  }
  break;

  case MQTT_EVENT_ERROR:
    Serial.println("[MQTT] Terjadi kesalahan koneksi/TLS.");
    break;

  default:
    break;
  }
}
void mqtt_app_start(bool isLocal)
{
  esp_mqtt_client_config_t mqtt_cfg = {};

  if (!isLocal)
  { // internet
    // Alamat lengkap beserta path WebSocket (/mqtt atau /ws tergantung broker)
    mqtt_cfg.uri = mqtt_broker_uri;

    // Verifikasi sertifikat SSL menggunakan bundle CA bawaan ESP32
    mqtt_cfg.use_global_ca_store = true;

    // Opsional: Jika broker membutuhkan username & password
    mqtt_cfg.username = "iot";
    mqtt_cfg.password = "iot123";
  }
  else
  { // local
    mqtt_cfg.uri = mqtt_broker_uri_local;
  }

  client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  esp_mqtt_client_start(client);
}
// Fungsi helper agar mudah digunakan di mana saja
void publishLogToMqtt(String message)
{
  if (serialLogQueue != NULL)
  {
    char buffer[128];
    message.toCharArray(buffer, sizeof(buffer));
    // Kirim ke queue dengan timeout 0 (non-blocking jika dipanggil dari interrupt/task cepat)
    xQueueSend(serialLogQueue, &buffer, 0);
  }
}
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
void readLDR(void *pvParameters)
{
  for (;;)
  {
    L1 = analogRead(LDR1);
    L2 = analogRead(LDR2);
    L3 = analogRead(LDR3);
    L4 = analogRead(LDR4);

    L1 = kalmanUpdate(L1);
    L2 = kalmanUpdate(L2);
    L3 = kalmanUpdate(L3);
    L4 = kalmanUpdate(L4);
    sum = L1 + L2 + L3 + L4 + 0.001;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void readINA()
{
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
  {
    busVoltage = INA.getBusVoltage();        // Tegangan beban (Volt)
    shuntVoltage = INA.getShuntVoltage_mV(); // Tegangan pada shunt (mV)
    current = INA.getCurrent_mA();           // Arus (milliampere)
    power = INA.getPower_mW();               // Daya (mW)

    // Tampilkan ke Serial Monitor
    // Serial.print("Tegangan Bus: "); Serial.print(busVoltage); Serial.println(" V");
    // Serial.print("Arus        : "); Serial.print(current); Serial.println(" mA");
    // Serial.print("Daya        : "); Serial.print(power); Serial.println(" mW");
    // Serial.println("----------------u-------------");

    xSemaphoreGive(i2cMutex); // UNLOCK
  }
}
void checkWifi(void *pvParameters)
{
  unsigned long timestart = millis();
  bool hasrun = false;
  for (;;)
  {
    unsigned long now = millis();
    if (WiFi.status() == WL_CONNECTED)
    {
      timestart = millis(); // reset waktu ketika wifi konek
      isWifiConnected = true;
      digitalWrite(LED_WIFI, HIGH);
      if (!hasrun)
      {
        // Mulai klien MQTT WSS
        mqtt_app_start(false);
        hasrun = true;
      }
    }
    else
    {
      isWifiConnected = false;
      static bool blinking = false;
      if (blinking)
      digitalWrite(LED_WIFI, LOW);
      else
      digitalWrite(LED_WIFI, HIGH);
      blinking = !blinking;
    }
    
    static bool hasrun1 = false;
    if (now - timestart >= 15000 && !hasrun1)
    {
      WiFi.disconnect(true);
      vTaskDelay(pdMS_TO_TICKS(500));
      // 2. Ubah mode sepenuhnya ke AP
      WiFi.mode(WIFI_AP);
      
      // 3. Jalankan AP
      WiFi.softAP("ESP32_Access_Point", "password123");
      
      Serial.print("Berhasil beralih ke AP Mode! IP: ");
      Serial.println(WiFi.softAPIP());
      mqtt_app_start(true);
      hasrun1 = true;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
void executeCommand(String cmd)
{
  cmd.toUpperCase();
  int sp = cmd.indexOf(' ');
  String verb = (sp == -1) ? cmd : cmd.substring(0, sp);
  String arg = (sp == -1) ? "" : cmd.substring(sp + 1);

  if (verb == "MANUAL")
  {
    arg.trim();
    if (arg == "ON")
    {
      manualMode = true;
      String msg = "Manual mode: ON";
      Serial.println(msg);
      publishLogToMqtt(msg);
    }
    else if (arg == "OFF")
    {
      manualMode = false;
      String msg = "Manual mode: OFF";
      Serial.println(msg);
      publishLogToMqtt(msg);
    }
    else
    {
      String msg = "Usage: MANUAL ON|OFF";
      Serial.println(msg);
      publishLogToMqtt(msg);
    }
  }
  else if (verb == "PAN" || verb == "TILT")
  {
    if (arg.length() == 0)
    {
      String msg = "Provide degree value, e.g. PAN 90";
      Serial.println(msg);
      publishLogToMqtt(msg);
    }
    else
    {
      int deg = arg.toInt();
      if (verb == "PAN")
      {
        int us = degToUs(deg, PAN_MIN, PAN_MAX);
        panAngle = us;
        servoPan.writeMicroseconds(clampInt(us, PAN_MIN, PAN_MAX));
        String msg = "PAN set: " + String(deg) + " deg -> " + String(us) + " us";
        Serial.println(msg);
        publishLogToMqtt(msg);
      }
      else
      {
        int us = tiltDegToUs(deg);
        tiltAngle = us;
        servoTilt.writeMicroseconds(clampInt(us, TILT_MIN, TILT_MAX));
        String msg = "TILT set: " + String(deg) + " deg -> " + String(us) + " us";
        Serial.println(msg);
        publishLogToMqtt(msg);
      }
    }
  }
  else if (verb == "PAN_US" || verb == "TILT_US")
  {
    if (arg.length() == 0)
    {
      String msg = "Provide microseconds value, e.g. PAN_US 1500";
      Serial.println(msg);
      publishLogToMqtt(msg);
    }
    else
    {
      int us = arg.toInt();
      if (verb == "PAN_US")
      {
        us = clampInt(us, PAN_MIN, PAN_MAX);
        panAngle = us;
        servoPan.writeMicroseconds(us);
        String msg = "PAN_US set: " + String(us) + " us";
        Serial.println(msg);
        publishLogToMqtt(msg);
      }
      else
      {
        us = clampInt(us, TILT_MIN, TILT_MAX);
        tiltAngle = us;
        servoTilt.writeMicroseconds(us);
        String msg = "TILT_US set: " + String(us) + " us";
        Serial.println(msg);
        publishLogToMqtt(msg);
      }
    }
  }
  else if (verb == "PAN_DELTA" || verb == "TILT_DELTA")
  {
    if (arg.length() == 0)
    {
      String msg = "Provide delta degrees, e.g. PAN_DELTA -5";
      Serial.println(msg);
      publishLogToMqtt(msg);
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
        String msg = "PAN_DELTA: now " + String(nd) + " deg -> " + String(us) + " us";
        Serial.println(msg);
        publishLogToMqtt(msg);
      }
      else
      {
        int curDeg = tiltUsToDeg((int)tiltAngle);
        int nd = clampInt(curDeg + d, TILT_DEG_MIN, TILT_DEG_MAX);
        int us = tiltDegToUs(nd);
        tiltAngle = us;
        servoTilt.writeMicroseconds(us);
        String msg = "TILT_DELTA: now " + String(nd) + " deg -> " + String(us) + " us";
        Serial.println(msg);
        publishLogToMqtt(msg);
      }
    }
  }
  else if (verb == "STEP")
  {
    if (arg.length() == 0)
    {
      String msg = "Current stepSize: " + String(stepSize);
      Serial.println(msg);
      publishLogToMqtt(msg);
    }
    else
    {
      int s = arg.toInt();
      if (s <= 0)
      {
        String msg = "Step must be > 0";
        Serial.println(msg);
        publishLogToMqtt(msg);
      }
      else
      {
        stepSize = s;
        String msg = "stepSize set to " + String(stepSize);
        Serial.println(msg);
        publishLogToMqtt(msg);
      }
    }
  }
  else if (verb == "HOME")
  {
    panAngle = (PAN_MIN + PAN_MAX) / 2;    // 1500 us
    tiltAngle = (TILT_MIN + TILT_MAX) / 2; // 1300 us
    servoPan.writeMicroseconds((int)panAngle);
    servoTilt.writeMicroseconds((int)tiltAngle);
    String msg = "Homing: pan -> " + String((int)panAngle) + " us | tilt -> " + String((int)tiltAngle) + " us";
    Serial.println(msg);
    publishLogToMqtt(msg);
  }
  else if (verb == "STATUS")
  {
    int panDeg = usToDeg((int)panAngle, PAN_MIN, PAN_MAX);
    int tiltDeg = tiltUsToDeg((int)tiltAngle);
    char buf[128];
    snprintf(buf, sizeof(buf), "Manual:%s | pan=%d us (%d deg) | tilt=%d us (%d deg) | step=%d",
             manualMode ? "ON" : "OFF", (int)panAngle, panDeg, (int)tiltAngle, tiltDeg, stepSize);
    Serial.println(buf);
    publishLogToMqtt(buf);
  }
  else if (verb == "READINA")
  {
    // readINA();
  }
  else if (verb == "READSENS")
  {
    char buf[128];
    snprintf(buf, sizeof(buf), "L1=%.0f | L2=%.0f | L3=%.0f | L4=%.0f | Sum=%.0f\n", L1, L2, L3, L4, sum);
    Serial.println(buf);
    publishLogToMqtt(buf);
  }
  else
  {
    Serial.println("Unknown command. Use MANUAL, PAN, TILT, PAN_DELTA, TILT_DELTA, PAN_US, TILT_US, STEP, HOME, STATUS, READINA, READSENS");
    publishLogToMqtt("Unknown command. Use MANUAL, PAN, TILT, PAN_DELTA, TILT_DELTA, PAN_US, TILT_US, STEP, HOME, STATUS, READINA, READSENS");
  }
}
void TaskReadDS18B20(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // Kita set task ini berjalan setiap 1 detik sekali (1000 ms)
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);

  bool conversionStarted = false;

  for (;;)
  {
    if (!conversionStarted)
    {
      // Tahap A: Perintahkan sensor mulai konversi suhu (tanpa menunggu/async)
      sensors.setWaitForConversion(false); // PENTING: Set false agar tidak blocking
      sensors.requestTemperatures();
      conversionStarted = true;
    }
    else
    {
      // Tahap B: Ambil hasil suhunya (karena sudah lewat 1 detik dari perintah sebelumnya)
      tempC = sensors.getTempCByIndex(0);

      // Lakukan sesuatu dengan nilai tempC (simpan ke variabel global, dll)

      // Mulai siklus konversi berikutnya
      sensors.requestTemperatures();
    }

    // Task tidur tenang selama 1 detik tanpa membebani CPU
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
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
          executeCommand(buf);
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
void TaskMqtt(void *pvParameters)
{
  char receivedLog[128];
  for (;;)
  {
    if (isMqttConnected && (millis() - lastPublish > 1000))
    {
      lastPublish = millis();

      readINA();
      // Cek apakah ada antrian log serial yang harus dikirim segera
      if (isMqttConnected && serialLogQueue != NULL)
      {
        // Ambil data dari queue jika ada
        while (xQueueReceive(serialLogQueue, &receivedLog, 0) == pdPASS)
        {
          esp_mqtt_client_publish(client, topic_pub_log, receivedLog, 0, 1, 0);
          vTaskDelay(pdMS_TO_TICKS(50)); // Jeda kecil antar publish agar tidak flood broker
        }
      }
      // 1. Topik INA226 (Tegangan, Arus, Daya)
      String payloadINA = "{\"bus_voltage\":" + String(busVoltage, 2) +
                          ",\"shunt_voltage\":" + String(shuntVoltage, 2) +
                          ",\"current\":" + String(current, 2) +
                          ",\"power\":" + String(power, 2) + "}";
      esp_mqtt_client_publish(client, "tracker/sensor/ina226", payloadINA.c_str(), 0, 1, 0);

      // 2. Topik Sensor LDR (L1 - L4)
      String payloadLDR = "{\"ldr1\":" + String(L1, 1) +
                          ",\"ldr2\":" + String(L2, 1) +
                          ",\"ldr3\":" + String(L3, 1) +
                          ",\"ldr4\":" + String(L4, 1) +
                          ",\"sum\":" + String(sum, 1) + "}";
      esp_mqtt_client_publish(client, "tracker/sensor/ldr", payloadLDR.c_str(), 0, 1, 0);

      // 3. Topik Sudut Tracker & Suhu
      int panDeg = usToDeg((int)panAngle, PAN_MIN, PAN_MAX);
      int tiltDeg = tiltUsToDeg((int)tiltAngle);
      String payloadStatus = "{\"pan_us\":" + String((int)panAngle) +
                             ",\"pan_deg\":" + String(panDeg) +
                             ",\"tilt_us\":" + String((int)tiltAngle) +
                             ",\"tilt_deg\":" + String(tiltDeg) +
                             ",\"temp_c\":" + String(tempC, 2) +
                             ",\"uptime\":" + String(millis() / 1000) + "}";
      esp_mqtt_client_publish(client, "tracker/status", payloadStatus.c_str(), 0, 1, 0);

      // Parameter: client, topic, data, len (0 = otomatis), qos, retain
      Serial.println("[MQTT] Semua data sensor berhasil dipublish ke topik terpisah.");
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
void tracking()
{
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

  servoPan.writeMicroseconds(panAngle);
  servoTilt.writeMicroseconds(tiltAngle);
}
void TaskTracking(void *pvParameters)
{
  for (;;)
  {
    if (!manualMode)
      tracking();

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void setup()
{
  Serial.begin(115200);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_WIFI, LOW);

  // Inisialisasi  I2C
  Serial.println("Initializing I2C buses...");
  Wire.begin(21, 22); // SDA, SCL
  delay(100);
  u8g2.begin();
  delay(100);
  u8g2.clearBuffer();                  // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
  u8g2.drawStr(0, 10, "Hello World!"); // write something to the internal memory
  u8g2.sendBuffer();                   // transfer internal memory to the display
  delay(100);

  // Buat queue dengan kapasitas 10 item, masing-masing berupa pointer char* atau String
  serialLogQueue = xQueueCreate(10, sizeof(char[128]));
  if (serialLogQueue == NULL)
  {
    Serial.println("Gagal membuat serialLogQueue!");
  }

  // Cek batas bawah (putus/korslet ke GND) dan batas atas (putus/korslet ke VCC)
  L1 = analogRead(LDR1);
  L2 = analogRead(LDR2);
  L3 = analogRead(LDR3);
  L4 = analogRead(LDR4);
  bool errorBawah = (L1 <= MIN_LDR || L2 <= MIN_LDR || L3 <= MIN_LDR || L4 <= MIN_LDR);
  bool errorAtas = (L1 >= MAX_LDR || L2 >= MAX_LDR || L3 >= MAX_LDR || L4 >= MAX_LDR);

  if (errorBawah || errorAtas)
  {
    Serial.println("Salah satu sensor bermasalah, periksa kabel!");
    while (1)
    { // Error Sensor dua kali kedip
      digitalWrite(LED_STATUS, HIGH);
      delay(100);
      digitalWrite(LED_STATUS, LOW);
      delay(100);
      digitalWrite(LED_STATUS, HIGH);
      delay(100);
      digitalWrite(LED_STATUS, LOW);
      delay(1000);
    }
  }

  // Inisialisasi sensor INA226
  if (!INA.begin())
  {
    Serial.println("Gagal mendeteksi sensor INA226! Periksa kabel.");
    while (1) // Error INA226 tiga kali kedip
    {
      digitalWrite(LED_STATUS, HIGH);
      delay(100);
      digitalWrite(LED_STATUS, LOW);
      delay(100);
      digitalWrite(LED_STATUS, HIGH);
      delay(100);
      digitalWrite(LED_STATUS, LOW);
      delay(100);
      digitalWrite(LED_STATUS, HIGH);
      delay(100);
      digitalWrite(LED_STATUS, LOW);
      delay(1000);
    }
  }

  // Kalibrasi arus maksimal (misal: 10 Ampere) dan nilai resistor shunt (misal: 0.1 Ohm)
  // Sesuaikan dengan nilai resistor shunt fisik pada modul Anda (umumnya 0.1 Ohm atau 0.002 Ohm)
  INA.setMaxCurrentShunt(10.0, 0.1);

  Serial.println("INA226 berhasil diinisialisasi.");

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

  /*********** BLYNK ***********/
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass); // connect to Wi-Fi

  // Core 1: Khusus untuk Kendali Utama & Komunikasi Data
  // Pindahkan tracking dan sensor cahaya ke sini agar tidak terganggu WiFi
  xTaskCreatePinnedToCore(readLDR, "readLDR", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskTracking, "Task Tracking", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskMqtt, "mqtt", 4096, NULL, 1, NULL, 1);

  // Core 0: Fokus ke Input Serial, Sistem, dan Sensor Lambat
  // Berbagi beban dengan internal WiFi stack
  xTaskCreatePinnedToCore(ReadString, "readString", 4096, NULL, 3, NULL, 0); // Priority 3 agar perintah cepat direspon
  xTaskCreatePinnedToCore(TaskReadDS18B20, "readTemp", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskOled, "Display Oled", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(checkWifi, "Check Wi-Fi", 4096, NULL, 1, NULL, 0); // Cukup Priority 1
  // Delete "setup and loop" task
  vTaskDelete(NULL);
}
void loop()
{
}