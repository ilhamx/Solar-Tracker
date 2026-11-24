#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <DallasTemperature.h>
#include "INA226.h"
#include <ESP32Servo.h>

// #include <WiFi.h>
// #include <ESPmDNS.h>
// #include <NetworkUdp.h>
// #include <ArduinoOTA.h>

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

SemaphoreHandle_t i2cMutex;

// Konfigurasi pin dll...
OneWire oneWire(23);
DallasTemperature ds18b20(&oneWire);

// Create an INA3221 object
INA226 INA(0x40);
// Queue handle untuk menampung string log (kapasitas 10 antrian)
QueueHandle_t serialLogQueue;

// char ssid[] = "ppk";
// char pass[] = "11111111";
// char ssid[] = "LPG_3KG";
// char pass[] = "piulbaikhati";

const char *ssid = "LPG_3KG";
const char *pass = "piulbaikhati";

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

/*********** SERVO ***********/
Servo servoPan;
Servo servoTilt;

#define YAW_LIMIT 160 // max rotation from start (degrees)

// Manual control flag: when true, automatic tracking (updateSens) is paused
volatile bool manualMode = false;

bool isMqttConnected = false;

// Helper functions untuk konversi dan clamp
inline int clampInt(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}
// Konversi derajat (0-180) ke microseconds antara minUs..maxUs
inline int degToUs(int deg, int minUs, int maxUs)
{
    deg = clampInt(deg, 0, 180);
    return map(deg, 0, 180, minUs, maxUs);
}
// Konversi microseconds ke derajat
inline int usToDeg(int us, int minUs, int maxUs)
{
    us = clampInt(us, minUs, maxUs);
    return map(us, minUs, maxUs, 0, 180);
}
inline int tiltDegToUs(int deg)
{
    deg = clampInt(deg, TILT_DEG_MIN, TILT_DEG_MAX);
    return map(deg, TILT_DEG_MIN, TILT_DEG_MAX, TILT_MIN, TILT_MAX);
}
inline int tiltUsToDeg(int us)
{
    us = clampInt(us, TILT_MIN, TILT_MAX);
    return map(us, TILT_MIN, TILT_MAX, TILT_DEG_MIN, TILT_DEG_MAX);
}