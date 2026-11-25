#include <Arduino.h>
#include <ArduinoOTA.h>
#include "utils.h"
#include "mqtt.h"
#include "oled.h"
#include "sensor.h"
#include "tracker.h"

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
        int us = degToUs(deg, PAN_MIN, PAN_MAX);
        track.panAngle = us;
        servoPan.writeMicroseconds(clampInt(us, PAN_MIN, PAN_MAX));
        String msg = "PAN set: " + String(deg) + " deg -> " + String(us) + " us";
        logInfo(msg);
      }
      else
      {
        int us = tiltDegToUs(deg);
        track.tiltAngle = us;
        servoTilt.writeMicroseconds(clampInt(us, TILT_MIN, TILT_MAX));
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
        us = clampInt(us, PAN_MIN, PAN_MAX);
        track.panAngle = us;
        servoPan.writeMicroseconds(us);
        String msg = "PAN_US set: " + String(us) + " us";
        logInfo(msg);
      }
      else
      {
        us = clampInt(us, TILT_MIN, TILT_MAX);
        track.tiltAngle = us;
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
        int curDeg = usToDeg((int)track.panAngle, PAN_MIN, PAN_MAX);
        int nd = clampInt(curDeg + d, 0, 180);
        int us = degToUs(nd, PAN_MIN, PAN_MAX);
        track.panAngle = us;
        servoPan.writeMicroseconds(us);
        String msg = "PAN_DELTA: now " + String(nd) + " deg -> " + String(us) + " us";
        logInfo(msg);
      }
      else
      {
        int curDeg = tiltUsToDeg((int)track.tiltAngle);
        int nd = clampInt(curDeg + d, TILT_DEG_MIN, TILT_DEG_MAX);
        int us = tiltDegToUs(nd);
        track.tiltAngle = us;
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
      String msg = "Current stepSize: " + String(track.stepSize);
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
        track.stepSize = s;
        String msg = "stepSize set to " + String(track.stepSize);
        logInfo(msg);
      }
    }
  }
  else if (verb == "HOME")
  {
    track.panAngle = (PAN_MIN + PAN_MAX) / 2;    // 1500 us
    track.tiltAngle = (TILT_MIN + TILT_MAX) / 2; // 1300 us
    servoPan.writeMicroseconds((int)track.panAngle);
    servoTilt.writeMicroseconds((int)track.tiltAngle);
    String msg = "Homing: pan -> " + String((int)track.panAngle) + " us | tilt -> " + String((int)track.tiltAngle) + " us";
    logInfo(msg);
  }
  else if (verb == "STATUS")
  {
    int panDeg = usToDeg((int)track.panAngle, PAN_MIN, PAN_MAX);
    int tiltDeg = tiltUsToDeg((int)track.tiltAngle);
    char buf[128];
    snprintf(buf, sizeof(buf), "Manual:%s | pan=%d us (%d deg) | tilt=%d us (%d deg) | step=%d",
             manualMode ? "ON" : "OFF", (int)track.panAngle, panDeg, (int)track.tiltAngle, tiltDeg, track.stepSize);
    logInfo(buf);
  }
  else if (verb == "SENSOR")
  {
    char buf[128];
    snprintf(buf, sizeof(buf), "L1=%.0f | L2=%.0f | L3=%.0f | L4=%.0f | Sum=%.0f\n", sensor.L1, sensor.L2, sensor.L3, sensor.L4, sensor.sumLDR);
    logInfo(buf);
  }
  else if (verb == "DEEPSLEEP")
  {
    if (arg.length() == 0)
    {
      String msg = "Provide time in hour, e.g. DEEPSLEEP 3";
      logInfo(msg);
    }
    else
    {
      int d = arg.toInt();
      String msg = "Memulai esp32 deepsleep selama " + String(d) + " jam...";
      logInfo(msg);
  
      vTaskDelay(pdMS_TO_TICKS(3000));
      // uint64_t sleep_seconds = 15; // 24 hours
      // uint64_t sleep_seconds = 3600ULL * 24;
      uint64_t sleep_seconds = 3600ULL * d;
      esp_sleep_enable_timer_wakeup(sleep_seconds * 1000000ULL);
      esp_deep_sleep_start();
    }
  }
  else if (verb == "REBOOT")
  {
    logInfo("Mulai reboot esp32 dalam 3 detik...");

    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP.restart();
  }
  else if (verb == "OTA")
  {
    logWarning("Mulai masuk mode OTA dalam 3 detik...");
    logWarning("Connect your PC to the ESP32 Wi-Fi.");

    vTaskDelay(pdMS_TO_TICKS(3000));
    otaTriggerMqtt = true;
  }
  else
  {
    logWarning("Unknown command. Use MANUAL, PAN, TILT, PAN_DELTA, TILT_DELTA, PAN_US, TILT_US, STEP, HOME, STATUS, SENSOR");
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
      sensor.tempC = ds18b20.getTempCByIndex(0);

      // Lakukan sesuatu dengan nilai tempC (simpan ke variabel global, dll)
      // String msg = "Suhu sudah diambil"
      // logInfo("suhu sudah diambil");
      // Mulai siklus konversi berikutnya
      ds18b20.requestTemperatures();
    }

    // Task tidur tenang selama 1 detik tanpa membebani CPU
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskMqtt(void *pvParameters)
{
  for (;;)
  {

    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.reconnect();
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    // Process MQTT and its WebSocket transport
    mqtt.update();

    // Detect disconnection
    if (wasConnected && !mqtt.isConnected())
    {
      Serial.println("[MQTT] Connection lost!");
      wasConnected = false;
    }

    // Reconnect MQTT
    if (!mqtt.isConnected())
    {
      connectMQTT();
      vTaskDelay(pdMS_TO_TICKS(3000));
      Serial.println("Reconnect MQTT");
    }

    // Publish
    if (mqtt.isConnected())
    {
      char buffer[128];
      static float voltage = 2.4, ampere = 0.1f;
      voltage += 0.1f;
      ampere += 0.1f;
      static uint i = 0;
      i++;
      // sprintf(buffer, "ONLINE ke %d", i);
      // mqtt.publish("solar/tracker/status", buffer);
      mqtt.publish("solar/ldr/l1", String(sensor.L1));
      mqtt.publish("solar/ldr/l2", String(sensor.L2));
      mqtt.publish("solar/ldr/l3", String(sensor.L3));
      mqtt.publish("solar/ldr/l4", String(sensor.L4));
      mqtt.publish("solar/temp/t1", String(sensor.tempC));
      mqtt.publish("solar/tracker/voltage", String(voltage));
      mqtt.publish("solar/tracker/ampere", String(ampere));
      // logInfo("berhasil kirim data");

      String payload = "{";
      payload += "\"voltage\":" + String(sensor.busVoltage, 2) + ",";
      payload += "\"current\":" + String(sensor.current, 2) + ",";
      payload += "\"power\":" + String(sensor.power, 2) + ",";
      payload += "\"temp\":" + String(sensor.tempC, 2) + ",";
      payload += "\"currentPan\":" + String(track.currentPan) + ",";
      payload += "\"currentTilt\":" + String(track.currentTilt) + ",";
      payload += "\"panAngle\":" + String(track.panAngle, 1) + ",";
      payload += "\"tiltAngle\":" + String(track.tiltAngle, 1);
      payload += "}";

      // Serial.printf("data panAngle: %.2f\n", track.panAngle);
      // Serial.printf("data mutex panAngle: %.2f\n", mqttTracker.panAngle);

      bool result = mqtt.publish(
          TOPIC_PUB,
          payload);

      Serial.println(
          result ? "[MQTT TX] Success" : "[MQTT TX] Failed");
    }
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
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(50);
  for (;;)
  {
    // if (manualMode || sensor.sumLDR < 50) // night protection
    if (manualMode) // night protection
    {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Update variabel tracker
    // Selesai mengakses track

    /*********** FUSION **********
     * Calculate normalized error in X and Y directions based on light levels
     * The idea is to find the imbalance in light intensity across the sensors
     */
    track.errX = (sensor.L1 + sensor.L2) - (sensor.L3 + sensor.L4);
    track.errY = (sensor.L1 + sensor.L3) - (sensor.L2 + sensor.L4);

    if (track.tiltAngle > 1500)
      track.nx = track.errX / track.sum;
    else
      track.nx = -track.errX / track.sum;
    track.ny = track.errY / track.sum;

    if (abs(track.nx) < track.deadzone)
      track.nx = 0;
    if (abs(track.ny) < track.deadzone)
      track.ny = 0;

    track.currentPan += track.nx * track.Kp_X;
    track.tiltAngle += track.ny * track.Kp_Y;

    track.currentPan = constrain(track.currentPan, PAN_MIN, PAN_MAX);
    track.tiltAngle = constrain(track.tiltAngle, TILT_MIN, TILT_MAX);

    static int panMinCounter = 0;
    if (track.currentPan <= PAN_MIN)
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
        track.currentPan = 1500;
        track.currentTilt = 1500;
        continue;
      }
    }
    else
    {
      panMinCounter = 0;
    }

    static int panMaxCounter = 0;

    if (track.currentPan >= PAN_MAX)
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
        track.currentPan = 1500;
        track.currentTilt = 1500;
        continue;
      }
    }
    else
    {
      panMaxCounter = 0;
    }
    xSemaphoreGive(trackerMutex);
    servoPan.writeMicroseconds(track.currentPan);
    servoTilt.writeMicroseconds(track.currentTilt);

    vTaskDelayUntil(&lastWakeTime, period);
  }
}
// FreeRTOS Task for handling OTA
void TaskOta(void *parameter)
{
  for (;;)
  {
    // 1. Check if button was pressed and OTA isn't running yet
    if ((otaTrigger || otaTriggerMqtt) && !otaActive)
    {
      otaTrigger = false;
      otaTriggerMqtt = false;
      otaActive = true;

      logWarning("Command Accepted! Starting AP mode and OTA...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      // Switch to Access Point mode
      WiFi.disconnect(true);
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid, ap_pass);

      Serial.print("AP IP Address: ");
      Serial.println(WiFi.softAPIP()); // Usually 192.168.4.1

      // Configure and start ArduinoOTA
      ArduinoOTA.setHostname("esp32");
      // ArduinoOTA.setPassword("admin");

      ArduinoOTA.onStart([]()
                         { Serial.println("OTA Update Starting..."); });
      ArduinoOTA.onEnd([]()
                       { Serial.println("\nOTA Complete! Rebooting..."); });
      ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                            { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
      ArduinoOTA.onError([](ota_error_t error)
                         { Serial.printf("Error[%u]\n", error); });

      ArduinoOTA.begin();
      Serial.println("OTA Ready. Connect your PC to the ESP32 Wi-Fi.");
    }

    // 2. Continuously handle OTA requests once active
    if (otaActive)
    {
      ArduinoOTA.handle();
    }
    // Yield to FreeRTOS scheduler (keeps watchdog happy)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
void setup()
{
  Serial.begin(115200);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  digitalWrite(LED_STATUS, LOW);
  digitalWrite(LED_WIFI, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);
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
  servoPan.writeMicroseconds(track.panAngle);
  servoTilt.writeMicroseconds(track.tiltAngle);

  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL)
  {
    Serial.println("Mutex gagal dibuat!");
  }
  trackerMutex = xSemaphoreCreateMutex();
  if (trackerMutex == nullptr)
  {
    Serial.println("Gagal membuat Mutex!");
    return;
  }
  Serial.println("Mutex berhasil dibuat!");

  /*********** MQTT ***********/
  WiFi.setAutoReconnect(true);
  connectWiFi();
  setupMQTT();
  // Core 1: Khusus untuk Kendali Utama & Komunikasi Data
  // Pindahkan tracking dan sensor cahaya ke sini agar tidak terganggu WiFi
  xTaskCreatePinnedToCore(taskSensor, "Task Sensor", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskTracking, "Task Tracking", 8000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskMqtt, "mqtt", 8096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskOta, "ota", 8192, NULL, 3, NULL, 1);

  // Core 0: Fokus ke Input Serial, Sistem, dan Sensor Lambat
  // Berbagi beban dengan internal WiFi stack
  xTaskCreatePinnedToCore(ReadString, "readString", 4096, NULL, 1, NULL, 0); // Priority 3 agar perintah cepat direspon
  xTaskCreatePinnedToCore(TaskReadDS18B20, "readTemp", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskOled, "Display Oled", 8000, NULL, 1, NULL, 0);
  // Delete "setup and loop" task
  vTaskDelete(NULL);
}
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
