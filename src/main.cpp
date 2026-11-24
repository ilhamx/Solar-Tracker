#include <Arduino.h>
#include "utils.h"
#include "mqtt.h"
#include "oled.h"
#include "tracker.h"
#include "ina_ldr.h"

void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.println(WiFi.localIP());
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
      logInfo(msg);
    }
    else if (arg == "OFF")
    {
      manualMode = false;
      String msg = "Manual mode: OFF";
      logInfo(msg);
    }
    else
    {
      String msg = "Usage: MANUAL ON|OFF";
      logInfo(msg);
    }
  }
  else if (verb == "PAN" || verb == "TILT")
  {
    if (arg.length() == 0)
    {
      String msg = "Provide degree value, e.g. PAN 90";
      logInfo(msg);
    }
    else
    {
      int deg = arg.toInt();
      if (verb == "PAN")
      {
        int us = tracker.degToUs(deg, PAN_MIN, PAN_MAX);
        panAngle = us;
        servoPan.writeMicroseconds(tracker.clampInt(us, PAN_MIN, PAN_MAX));
        String msg = "PAN set: " + String(deg) + " deg -> " + String(us) + " us";
        logInfo(msg);
      }
      else
      {
        int us = tracker.tiltDegToUs(deg);
        tiltAngle = us;
        servoTilt.writeMicroseconds(tracker.clampInt(us, TILT_MIN, TILT_MAX));
        String msg = "TILT set: " + String(deg) + " deg -> " + String(us) + " us";
        logInfo(msg);
      }
    }
  }
  else if (verb == "PAN_US" || verb == "TILT_US")
  {
    if (arg.length() == 0)
    {
      String msg = "Provide microseconds value, e.g. PAN_US 1500";
      logInfo(msg);
    }
    else
    {
      int us = arg.toInt();
      if (verb == "PAN_US")
      {
        us = tracker.clampInt(us, PAN_MIN, PAN_MAX);
        panAngle = us;
        servoPan.writeMicroseconds(us);
        String msg = "PAN_US set: " + String(us) + " us";
        logInfo(msg);
      }
      else
      {
        us = tracker.clampInt(us, TILT_MIN, TILT_MAX);
        tiltAngle = us;
        servoTilt.writeMicroseconds(us);
        String msg = "TILT_US set: " + String(us) + " us";
        logInfo(msg);
      }
    }
  }
  else if (verb == "PAN_DELTA" || verb == "TILT_DELTA")
  {
    if (arg.length() == 0)
    {
      String msg = "Provide delta degrees, e.g. PAN_DELTA -5";
      logInfo(msg);
    }
    else
    {
      int d = arg.toInt();
      if (verb == "PAN_DELTA")
      {
        int curDeg = tracker.usToDeg((int)panAngle, PAN_MIN, PAN_MAX);
        int nd = tracker.clampInt(curDeg + d, 0, 180);
        int us = tracker.degToUs(nd, PAN_MIN, PAN_MAX);
        panAngle = us;
        servoPan.writeMicroseconds(us);
        String msg = "PAN_DELTA: now " + String(nd) + " deg -> " + String(us) + " us";
        logInfo(msg);
      }
      else
      {
        int curDeg = tracker.tiltUsToDeg((int)tiltAngle);
        int nd = tracker.clampInt(curDeg + d, TILT_DEG_MIN, TILT_DEG_MAX);
        int us = tracker.tiltDegToUs(nd);
        tiltAngle = us;
        servoTilt.writeMicroseconds(us);
        String msg = "TILT_DELTA: now " + String(nd) + " deg -> " + String(us) + " us";
        logInfo(msg);
      }
    }
  }
  else if (verb == "STEP")
  {
    if (arg.length() == 0)
    {
      String msg = "Current stepSize: " + String(stepSize);
      logInfo(msg);
    }
    else
    {
      int s = arg.toInt();
      if (s <= 0)
      {
        String msg = "Step must be > 0";
        logInfo(msg);
      }
      else
      {
        stepSize = s;
        String msg = "stepSize set to " + String(stepSize);
        logInfo(msg);
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
    logInfo(msg);
  }
  else if (verb == "STATUS")
  {
    int panDeg = tracker.usToDeg((int)panAngle, PAN_MIN, PAN_MAX);
    int tiltDeg = tracker.tiltUsToDeg((int)tiltAngle);
    char buf[128];
    snprintf(buf, sizeof(buf), "Manual:%s | pan=%d us (%d deg) | tilt=%d us (%d deg) | step=%d",
             manualMode ? "ON" : "OFF", (int)panAngle, panDeg, (int)tiltAngle, tiltDeg, stepSize);
    logInfo(buf);
  }
  else if (verb == "READINA")
  {
    sensor.readINA();
  }
  else if (verb == "READSENS")
  {
    char buf[128];
    snprintf(buf, sizeof(buf), "L1=%.0f | L2=%.0f | L3=%.0f | L4=%.0f | Sum=%.0f\n", L1, L2, L3, L4, sum);
    logInfo(buf);
  }
  else
  {
    logWarning("Unknown command. Use MANUAL, PAN, TILT, PAN_DELTA, TILT_DELTA, PAN_US, TILT_US, STEP, HOME, STATUS, READINA, READSENS");
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
      ds18b20.setWaitForConversion(false); // PENTING: Set false agar tidak blocking
      ds18b20.requestTemperatures();
      conversionStarted = true;
    }
    else
    {
      // Tahap B: Ambil hasil suhunya (karena sudah lewat 1 detik dari perintah sebelumnya)
      tempC = ds18b20.getTempCByIndex(0);

      // Lakukan sesuatu dengan nilai tempC (simpan ke variabel global, dll)

      // Mulai siklus konversi berikutnya
      ds18b20.requestTemperatures();
    }

    // Task tidur tenang selama 1 detik tanpa membebani CPU
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskMqtt(void *pvParameters)
{
  char receivedLog[128];
  static bool hasrun = false;
  for (;;)
  {
    loopMQTT();
    vTaskDelay(PUBLISH_INTERVAL / portTICK_PERIOD_MS);
  }
}
void taskSensor(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(2000));
  for (;;)
  {
    sensor.readLDR();
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
    {
      sensor.readINA();
      xSemaphoreGive(i2cMutex); // UNLOCK
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
void taskOled(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(2000));
  for (;;)
  {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY))
    {
      oled.drawSolar();
      xSemaphoreGive(i2cMutex); // UNLOCK
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
void TaskTracking(void *pvParameters)
{
  for (;;)
  {
    if (!manualMode)
      tracker.tracking();
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
  oled.begin();

  // Buat queue dengan kapasitas 10 item, masing-masing berupa pointer char* atau String
  serialLogQueue = xQueueCreate(10, sizeof(char[128]));
  if (serialLogQueue == NULL)
  {
    Serial.println("Gagal membuat serialLogQueue!");
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
  Serial.println("Mencoba menyambung ke WiFi...");
  WiFi.setAutoReconnect(true);
  connectWiFi();
  setupMQTT();
  // Core 1: Khusus untuk Kendali Utama & Komunikasi Data
  // Pindahkan tracking dan sensor cahaya ke sini agar tidak terganggu WiFi
  // xTaskCreatePinnedToCore(taskSensor, "Task Sensor", 4096, NULL, 2, NULL, 1);
  // xTaskCreatePinnedToCore(TaskTracking, "Task Tracking", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskMqtt, "mqtt", 8096, NULL, 1, NULL, 1);

  // Core 0: Fokus ke Input Serial, Sistem, dan Sensor Lambat
  // Berbagi beban dengan internal WiFi stack
  // xTaskCreatePinnedToCore(ReadString, "readString", 4096, NULL, 3, NULL, 0); // Priority 3 agar perintah cepat direspon
  // xTaskCreatePinnedToCore(TaskReadDS18B20, "readTemp", 4096, NULL, 1, NULL, 0);
  // xTaskCreatePinnedToCore(taskOled, "Display Oled", 4096, NULL, 1, NULL, 0);
  // xTaskCreatePinnedToCore(handleWifi, "Check Wi-Fi", 8096, NULL, 2, NULL, 1); // Cukup Priority 1
  // Delete "setup and loop" task
  // vTaskDelete(NULL);
}
void loop()
{
}