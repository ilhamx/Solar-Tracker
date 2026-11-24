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


// char ssid[] = "ppk";
// char pass[] = "11111111";
// char ssid[] = "LPG_3KG";
// char pass[] = "piulbaikhati";

const char* ssid = "LPG_3KG";
const char* pass = "piulbaikhati";

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
int currentPan = 600;        // posisi awal servo (microseconds)
int currentTilt = 1500;      // posisi awal servo (microseconds)
float L1, L2, L3, L4, tempC; // light levels from 4 sensors
float busVoltage, shuntVoltage, current, power;
float sum, nx, ny;                                // for fusion calculations
float L1_kalman, L2_kalman, L3_kalman, L4_kalman; // kalman-filtered light levels

#define YAW_LIMIT 160 // max rotation from start (degrees)

/*********** SERVO ***********/
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

bool isMqttConnected = false;

