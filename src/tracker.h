#pragma once

struct
{
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
} tracker;