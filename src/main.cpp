/*********** BLYNK ***********/

#include <Arduino.h>
#include <INA3221.h>

INA3221 ina(0x40);

// Blynk setup
#define BLYNK_TEMPLATE_ID "TMPL6eR1bVog2"
#define BLYNK_TEMPLATE_NAME "Solar Tracker"
#define BLYNK_AUTH_TOKEN "XOPuG9s9pjBzVcIWq-BsI5d6jrSCh5Cq"

#define BLYNK_PRINT Serial

#define Wire I2CBus1
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "URT";
char pass[] = "unimusrobotic";

BlynkTimer timer; // Create a timer object to handle timed events

/*********** I2C ***********/
#include <Wire.h>
TwoWire I2CBus1 = TwoWire(0); // default
TwoWire I2CBus2 = TwoWire(1); // second bus

/*********** MPU6050 ***********/
#include <MPU6050.h>
MPU6050 mpu;

float yaw0 = 0;
float currentYaw = 0;

#define YAW_LIMIT 160 // max rotation from start (degrees)

/*********** BH1750 ***********/
#include <BH1750.h>

BH1750 bh1; // Bus1 addr 0x23
BH1750 bh2; // Bus1 addr 0x5C
BH1750 bh3; // Bus2 addr 0x23
BH1750 bh4; // Bus2 addr 0x5C

/*********** SERVO ***********/
#include <ESP32Servo.h>
Servo servoPan;
Servo servoTilt;

#define PAN_PIN 26
#define TILT_PIN 27

float panAngle = 90;
float tiltAngle = 90;

#define PAN_MIN 50
#define PAN_MAX 140
#define TILT_MIN 55
#define TILT_MAX 125

// global variables for ina3221
float shuntV0, busV0, current0;
float shuntV1, busV1, current1;
float shuntV2, busV2, current2;

/*********** TRACKER PARAM ***********/
float Kp_X = 2.0;
float Kp_Y = 1.0;
float deadzone = 0.02;
unsigned long lastUpdate = 0;

float getYaw()
{
  static float angle = 0;
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  float gz_dps = gz / 131.0; // convert to deg/s
  angle += gz_dps * 0.5;     // 0.5 sec loop time

  return angle;
}

void myTimerEvent()
{
  /*********** READ SENSORS ***********/
  // Serial.println("reading sensor");
  float L1 = bh1.readLightLevel(); // top-left
  float L3 = bh2.readLightLevel(); // top-right
  float L2 = bh3.readLightLevel(); // bottom-left
  float L4 = bh4.readLightLevel(); // bottom-right

  float sum = L1 + L2 + L3 + L4 + 0.001;

  if (sum < 50) // night protection
  {
    panAngle = 90;
    tiltAngle = 90;
    servoPan.write(panAngle);
    servoTilt.write(tiltAngle);
    return;
  }

  /*********** FUSION ***********/
  float errX = (L2 + L4) - (L1 + L3);
  float errY = (L1 + L2) - (L3 + L4);

  float nx = errX / sum;
  float ny = errY / sum;

  if (abs(nx) < deadzone)
    nx = 0;
  if (abs(ny) < deadzone)
    ny = 0;

  /*********** CONTROL ***********/
  panAngle += nx * Kp_X; // multiplied by 10 to increase sensitivity
  tiltAngle += ny * Kp_Y;

  currentYaw = getYaw();

  float deltaYaw = currentYaw - yaw0;

  /*********** CONTROL WITH YAW PROTECTION ***********/
  if ((deltaYaw < YAW_LIMIT && nx > 0) ||
      (deltaYaw > -YAW_LIMIT && nx < 0))
  {
    panAngle += nx * Kp_X;
  }

  tiltAngle += ny * Kp_Y;

  panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
  tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);

  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  /*********** BLYNK ***********/
  // Serial.println("sending data to blynk.");
  // Blynk.virtualWrite(V0, sum);
  // Blynk.virtualWrite(V1, nx);
  // Blynk.virtualWrite(V2, ny);
  // Blynk.virtualWrite(V3, panAngle);
  // Blynk.virtualWrite(V4, tiltAngle);
  BLYNK_LOG("L1Lux %.0f | L2Lux %.0f | L3Lux %.0f | L4Lux %.0f | Sum %.0f | nx %.3f ny %.3f | Pan %.1f Tilt %.1f | Yaw %.1f", L1, L2, L3, L4, sum, nx, ny, panAngle, tiltAngle, currentYaw);
}

void readINA()
{
  shuntV0 = ina.getShuntVoltage(0);
  busV0 = ina.getBusVoltage(0);
  current0 = ina.getCurrent(0);

  shuntV1 = ina.getShuntVoltage(1);
  busV1 = ina.getBusVoltage(1);
  current1 = ina.getCurrent(1);

  shuntV2 = ina.getShuntVoltage(2);
  busV2 = ina.getBusVoltage(2);
  current2 = ina.getCurrent(2);

  if (isnan(shuntV0) || isnan(busV0) || isnan(current0) ||
      isnan(shuntV1) || isnan(busV1) || isnan(current1) ||
      isnan(shuntV2) || isnan(busV2) || isnan(current2))
  {
    BLYNK_LOG("Failed to read INA3221");
    return;
  }
  else
  {
    BLYNK_LOG("CH1: Shunt %.3f mV | Bus %.3f V | Current %.3f mA", shuntV0, busV0, current0);
    BLYNK_LOG("CH2: Shunt %.3f mV | Bus %.3f V | Current %.3f mA", shuntV1, busV1, current1);
    BLYNK_LOG("CH3: Shunt %.3f mV | Bus %.3f V | Current %.3f mA", shuntV2, busV2, current2);
  }
}

void sendBlynk()
{
  Blynk.virtualWrite(V0, panAngle);
  Blynk.virtualWrite(V1, tiltAngle);
  Blynk.virtualWrite(V2, busV0);
  Blynk.virtualWrite(V3, current0);
  Blynk.virtualWrite(V4, busV1);
  Blynk.virtualWrite(V5, current1);
  Blynk.virtualWrite(V6, busV2);
  Blynk.virtualWrite(V7, current2);
}

void setup()
{
  Serial.begin(115200);

  /*********** INIT I2C ***********/
  Serial.println("Initializing I2C buses...");
  I2CBus1.begin(21, 22, 400000); // SDA, SCL
  I2CBus2.begin(16, 17, 400000);

  /*********** INIT BH1750 ***********/
  Serial.println("Initializing BH1750 sensors...");
  bh1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2CBus1);
  bh2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &I2CBus1);
  bh3.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2CBus2);
  bh4.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &I2CBus2);

  /*********** SERVO ***********/
  Serial.println("Initializing servos...");
  servoPan.attach(PAN_PIN);
  servoTilt.attach(TILT_PIN);
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  // Ina3221
  Serial.println("Initializing INA3221...");
  ina.begin(&I2CBus1);
  ina.setMode(0x7127); // default mode: all channels enabled, 4 averages, 1.1ms conversion time

  ina.setShuntR(0, 0.1); // CH1 0.1 ohm (ubah sesuai hardware)
  ina.setShuntR(1, 0.1); // CH2
  ina.setShuntR(2, 0.1); // CH3

  /*********** MPU6050 ***********/
  Serial.println("Initializing MPU6050...");
  delay(100);
  mpu.initialize();
  if (!mpu.testConnection())
  {
    Serial.println("MPU6050 NOT CONNECTED!");
    while (1)
      ;
  }
  else
  {
    Serial.println("MPU6050 Connected");
  }

  delay(1000);

  // simple yaw zeroing
  yaw0 = 0;

  /*********** BLYNK ***********/
  Serial.println("Connecting to WiFi...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(200L, myTimerEvent);
  timer.setInterval(1000L, readINA);   // read INA3221 every 200 ms
  timer.setInterval(3000L, sendBlynk); // send data to Blynk every 3 second (adjust as needed)
  // timer.setInterval(1000L, check);
}

void loop()
{
  Blynk.run();
  timer.run();

  // if (millis() - lastUpdate < 500) return; // 2 Hz
  // lastUpdate = millis();
}
