#pragma once
#include "ina_ldr.h"
#include <WebSocketsClient.h>
#include <MQTTPubSubClient.h>
extern void executeCommand(String cmd);

// ========================================
// WIFI
// ========================================
const char *WIFI_SSID = "LPG_3KG";
const char *WIFI_PASS = "piulbaikhati";

// ========================================
// MQTT WEBSOCKET
// ========================================
const char *MQTT_HOST = "mqtt.solar2axis.my.id";
const uint16_t MQTT_PORT = 443;
const char *MQTT_PATH = "/mqtt";

const char *MQTT_USER = "iot";
const char *MQTT_PASS = "iot123";

const char *CLIENT_ID = "ESP32_SOLAR_TRACKER";
// ========================================
// TOPICS
// ========================================
const char *TOPIC_PUB = "solar/tracker/data";
const char *TOPIC_SUB = "solar/tracker/control";
const char *TOPIC_STATUS = "solar/tracker/status";
// ========================================
// CLIENT
// ========================================
WebSocketsClient ws;
MQTTPubSubClient mqtt;

unsigned long lastReconnect = 0;
unsigned long lastPublish = 0;

const unsigned long RECONNECT_INTERVAL = 5000;
const unsigned long PUBLISH_INTERVAL = 1000;

bool wasConnected = false;

// Helper Logging for mqtt
// ========================================
// MQTT LOGGING
// ========================================
void logMQTT(const String& message)
{
    // Serial Monitor lokal
    Serial.println(message);

    // MQTT Monitor
    if (mqtt.isConnected())
    {
        mqtt.publish("solar/tracker/log", message);
    }
}

// ========================================
// LOG LEVEL
// ========================================
void logInfo(const String& message)
{
    logMQTT(String("[INFO] ") + message);
}

void logWarning(const String& message)
{
    logMQTT(String("[WARNING] ") + message);
}

void logError(const String& message)
{
    logMQTT(String("[ERROR] ") + message);
}
// test
void publish()
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
}
// ========================================
// MQTT CALLBACK
// ========================================
void onMQTTMessage(
    const String &payload,
    const size_t size)
{
    Serial.print("[MQTT RX] ");
    Serial.println(payload);

    executeCommand(payload);
    // if (payload == "ON")
    // {
    //     Serial.println("Solar tracker ON");
    //     logWarning("Solar tracker ON");
    // }
    // else if (payload == "OFF")
    // {
    //     Serial.println("Solar tracker OFF");
    //     logWarning("Solar tracker OFF");
    // }
}

// ========================================
// WEBSOCKET EVENT
// ========================================
void onWebSocketEvent(
    WStype_t type,
    uint8_t *payload,
    size_t length)
{
    switch (type)
    {

    case WStype_CONNECTED:
        Serial.println("[WS] Connected!");
        break;

    case WStype_DISCONNECTED:
        Serial.println("[WS] Disconnected!");
        break;

    case WStype_ERROR:
        Serial.println("[WS] Error!");
        break;

    default:
        break;
    }
}

// ========================================
// MQTT CONNECT
// ========================================
void connectMQTT()
{

    Serial.println("[MQTT] Connecting...");

    if (mqtt.connect(
            CLIENT_ID,
            MQTT_USER,
            MQTT_PASS))
    {

        Serial.println("[MQTT] Connected!");

        mqtt.subscribe(
            TOPIC_SUB,
            onMQTTMessage);

        mqtt.publish(
            TOPIC_STATUS,
            "[INFO] SOLAR TRACKER IS ONLINE");

        wasConnected = true;
    }
    else
    {

        Serial.print("[MQTT] Failed: ");
        Serial.println((int)mqtt.getLastError());

        Serial.print("[MQTT] Return code: ");
        Serial.println((int)mqtt.getReturnCode());

        wasConnected = false;
    }
}
// ========================================
// SETUP
// ========================================
void setupMQTT()
{

    Serial.begin(115200);

    // WebSocket event
    ws.onEvent(onWebSocketEvent);

    // Initialize WSS FIRST
    ws.beginSSL(
        MQTT_HOST,
        MQTT_PORT,
        MQTT_PATH,
        nullptr,
        "mqtt");

    ws.setReconnectInterval(5000);

    // Initialize MQTT AFTER WebSocket
    mqtt.begin(ws);

    mqtt.setTimeout(10000);

    Serial.println("[SYSTEM] WSS initialized");

    // Initial MQTT connection
    connectMQTT();
}

// ========================================
// LOOP
// ========================================
void loopMQTT()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        WiFi.reconnect();
        vTaskDelay(pdMS_TO_TICKS(3000));
        return;
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

        String payload =
            "{\"temperature\":30.5,"
            "\"voltage\":12.4}";

        bool result = mqtt.publish(
            TOPIC_PUB,
            payload);
        publish();

        Serial.println(
            result ? "[MQTT TX] Success" : "[MQTT TX] Failed");
    }
}
