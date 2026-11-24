#pragma once
#include "ina_ldr.h"

struct trackerData
{
    float errX, errY;
    float sum, nx, ny; // for fusion calculations
    /*********** CONTROL ***********/
    float Kp_X = 7.0f; // adjust as needed
    float Kp_Y = 5.0f; // adjust as needed
    float deadzone = 0.08;
    unsigned long lastUpdate = 0;
    bool ledStatus = false, isWifiConnected = false;
    int stepSize = 1;  // kecepatan gerak (semakin kecil = semakin halus)
    int deadband = 10; // toleransi biar gak jitter
    float yaw0 = 0;
    float currentYaw = 0;
    int currentPan = 600;   // posisi awal servo (microseconds)
    int currentTilt = 1500; // posisi awal servo (microseconds)

    float L1_kalman, L2_kalman, L3_kalman, L4_kalman; // kalman-filtered light levels
    float panAngle = 1500;
    float tiltAngle = 1500;

    inline float kalmanUpdate(float measurement)
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
    inline void writePan(int targetUs)
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
    void protection()
    {
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
        errX = (sensor.L1 + sensor.L2) - (sensor.L3 + sensor.L4);
        errY = (sensor.L1 + sensor.L3) - (sensor.L2 + sensor.L4);

        if (tiltAngle > 1500)
            nx = errX / sum;
        else
            nx = -errX / sum;
        ny = errY / sum;

        if (abs(nx) < deadzone)
            nx = 0;
        if (abs(ny) < deadzone)
            ny = 0;

        panAngle += nx * Kp_X;
        tiltAngle += ny * Kp_Y;

        panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
        tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);

        protection();
        servoPan.writeMicroseconds(panAngle);
        servoTilt.writeMicroseconds(tiltAngle);
    }
} tracker;