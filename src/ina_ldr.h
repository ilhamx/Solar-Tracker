#pragma once
#include "tracker.h"

struct sensorData
{
    uint16_t L1, L2, L3, L4, sum;
    float tempC; // light levels from 4 sensors
    float busVoltage, shuntVoltage, current, power;
    
    float kalmanUpdate(float measurement, uint8_t index)
    {
        const float Q = 0.01;
        const float R = 1.0;
        
        static float x[4] = {0, 0, 0, 0};
        static float P[4] = {1, 1, 1, 1};

        P[index] += Q;

        float K = P[index] / (P[index] + R);

        x[index] += K * (measurement - x[index]);

        P[index] *= (1 - K);

        return x[index];
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
    inline void begin()
    {
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
    }
    void readLDR()
    {
        L1 = kalmanUpdate(analogRead(LDR1), 0);
        L2 = kalmanUpdate(analogRead(LDR2), 1);
        L3 = kalmanUpdate(analogRead(LDR3), 2);
        L4 = kalmanUpdate(analogRead(LDR4), 3);
        sum = L1 + L2 + L3 + L4;
    }
};

extern sensorData sensor;