#pragma once

struct
{
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
        L1 = analogRead(LDR1);
        L2 = analogRead(LDR2);
        L3 = analogRead(LDR3);
        L4 = analogRead(LDR4);

        L1 = tracker.kalmanUpdate(L1);
        L2 = tracker.kalmanUpdate(L2);
        L3 = tracker.kalmanUpdate(L3);
        L4 = tracker.kalmanUpdate(L4);
        sum = L1 + L2 + L3 + L4 + 0.001;
    }
} sensor;