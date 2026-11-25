#pragma once
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
#include <cstdarg>
#include <cstdio>
#include <new>

// Definisi konstanta Log Level (sesuaikan jika sudah ada di kode Anda)
enum LogLevel : uint8_t {
    LOG_DEBUG = 0,
    LOG_INFO  = 1,
    LOG_WARN  = 2,
    LOG_ERROR = 3
};

// Opsional: Filter batas minimum log yang ingin ditampilkan
uint8_t currentLogLevel = LOG_DEBUG; 

size_t logMQTT(uint8_t logLevel, const char *format, ...) {
    // Abaikan jika logLevel di bawah batas minimum yang ditentukan
    if (logLevel < currentLogLevel) {
        return 0;
    }

    // Tentukan prefix berdasarkan logLevel
    const char* prefix = "";
    switch (logLevel) {
        case LOG_DEBUG: prefix = "[DEBUG] "; break;
        case LOG_INFO:  prefix = "[INFO] ";  break;
        case LOG_WARN:  prefix = "[WARN] ";  break;
        case LOG_ERROR: prefix = "[ERROR] "; break;
        default:        prefix = "[LOG] ";   break;
    }
    size_t prefixLen = strlen(prefix);

    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;

    // Hitung panjang string pesan utama
    int msgLen = vsnprintf(NULL, 0, format, arg);
    va_end(arg);

    if (msgLen < 0) {
        return 0;
    }

    size_t totalLen = prefixLen + (size_t)msgLen;

    // Alokasi memori dinamis jika gabungan prefix + pesan melebihi 63 karakter
    if (totalLen > sizeof(temp) - 1) {
        buffer = new (std::nothrow) char[totalLen + 1];
        if (!buffer) {
            return 0; // Kehabisan RAM
        }
    }

    // Salin prefix ke awal buffer, lalu format pesan tepat setelah prefix
    memcpy(buffer, prefix, prefixLen);
    va_start(arg, format);
    vsnprintf(buffer + prefixLen, msgLen + 1, format, arg);
    va_end(arg);

    // 1. Kirim ke Serial Monitor lokal
    Serial.println(buffer);

    // 2. Kirim ke MQTT Monitor
    if (mqtt.isConnected()) {
        mqtt.publish("solar/tracker/log", String(buffer));
    }

    // Bersihkan memori jika menggunakan alokasi dinamis
    if (buffer != temp) {
        delete[] buffer;
    }

    return totalLen;
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