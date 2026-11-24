#pragma once

struct trackerData
{
    float errX, errY;
    float sum, nx, ny; // for fusion calculations
    /*********** CONTROL ***********/
    float Kp_X = 7.0f; // adjust as needed
    float Kp_Y = 5.0f; // adjust as needed
    float deadzone = 0.08;
    unsigned long lastUpdate = 0;
    bool ledStatus = false, isWifiConnected = false, hasrun = false;
    int stepSize = 1;  // kecepatan gerak (semakin kecil = semakin halus)
    int deadband = 10; // toleransi biar gak jitter
    float yaw0 = 0;
    float currentYaw = 0;
    int currentPan = 600;   // posisi awal servo (microseconds)
    int currentTilt = 1500; // posisi awal servo (microseconds)

    float panAngle = 20.0;
    float tiltAngle = 90.0;
} track;

SemaphoreHandle_t trackerMutex = nullptr;