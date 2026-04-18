/*********** BLYNK ***********/

#include <Arduino.h>
#include <INA3221.h>

INA3221 ina(0x40);

// Blynk setup
#define BLYNK_TEMPLATE_ID "TMPL6eR1bVog2"
#define BLYNK_TEMPLATE_NAME "Solar Tracker"
#define BLYNK_AUTH_TOKEN "XOPuG9s9pjBzVcIWq-BsI5d6jrSCh5Cq"

#define BLYNK_PRINT Serial

// #define Wire I2CBus1
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "URT";
char pass[] = "unimusrobotic";

BlynkTimer timer; // Create a timer object to handle timed events

/*********** I2C ***********/
#include <Wire.h>
// TwoWire I2CBus1 = TwoWire(0); // default
TwoWire I2CBus2 = TwoWire(1); // second bus

/*********** MPU6050 ***********/
#include <MPU6050.h>
MPU6050 mpu;

float yaw0 = 0;
float currentYaw = 0;
float L1, L2, L3, L4;                             // light levels from 4 sensors
float L1_kalman, L2_kalman, L3_kalman, L4_kalman; // kalman-filtered light levels
float sum, nx, ny;                                // for fusion calculations

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

#define PAN_MIN 70
#define PAN_MAX 110
#define TILT_MIN 55
#define TILT_MAX 125

// global variables for ina3221
float shuntV0, busV0, current0;
float shuntV1, busV1, current1;
float shuntV2, busV2, current2;

/*********** TRACKER PARAM ***********/
float Kp_X = 2.0; // pan is more sensitive to error, so we use a larger Kp
float Kp_Y = 3.0; // tilt is less sensitive to error, so we use a smaller Kp
float deadzone = 0.02;
unsigned long lastUpdate = 0;

// float getYaw()
// {
//   static float angle = 0;
//   int16_t gx, gy, gz;
//   mpu.getRotation(&gx, &gy, &gz);

//   float gz_dps = gz / 131.0; // convert to deg/s
//   angle += gz_dps * 0.5;     // 0.5 sec loop time

//   return angle;
// }

float kalmanUpdate(float measurement)
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

void updateSens()
{
  /*********** READ SENSORS ***********/
  // Serial.println("reading sensor");
  L1 = bh1.readLightLevel(); // top-left
  L3 = bh2.readLightLevel(); // top-right
  L2 = bh3.readLightLevel(); // bottom-left
  L4 = bh4.readLightLevel(); // bottom-right

  L1_kalman = kalmanUpdate(L1); // example of using Kalman filter on one sensor (can be applied to all);
  L2_kalman = kalmanUpdate(L2); // example of using Kalman filter on one sensor (can be applied to all);
  L3_kalman = kalmanUpdate(L3); // example of using Kalman filter on one sensor (can be applied to all);
  L4_kalman = kalmanUpdate(L4); // example of using Kalman filter on one sensor (can be applied to all);

  sum = L1 + L2 + L3 + L4 + 0.001;

  if (sum < 100) // night protection
  {
    panAngle = 90;
    tiltAngle = 90;
    servoPan.write(panAngle);
    servoTilt.write(tiltAngle);
    return;
  }

  /*********** FUSION **********
   * Calculate normalized error in X and Y directions based on light levels
   * The idea is to find the imbalance in light intensity across the sensors
   */
  float errX = (L1 + L2) - (L3 + L4);
  float errY = (L1 + L3) - (L2 + L4);

  nx = -errX / sum;
  ny = errY / sum;

  if (abs(nx) < deadzone)
    nx = 0;
  if (abs(ny) < deadzone)
    ny = 0;

  /*********** CONTROL ***********/
  panAngle += nx * Kp_X; // multiplied by 10 to increase sensitivity
  tiltAngle += ny * Kp_Y;

  // currentYaw = getYaw();

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

  /*********** BLYNK ***********/
  // Serial.println("sending data to blynk.");
  // Blynk.virtualWrite(V0, sum);
  // Blynk.virtualWrite(V1, nx);
  // Blynk.virtualWrite(V2, ny);
  // Blynk.virtualWrite(V3, panAngle);
  // Blynk.virtualWrite(V4, tiltAngle);
}

void updateServo()
{
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);
}

void readINA()
{
  shuntV0 = ina.getShuntVoltage(0);
  busV0 = ina.getBusVoltage(0);
  current0 = -ina.getCurrent(0);

  shuntV1 = ina.getShuntVoltage(1);
  busV1 = ina.getBusVoltage(1);
  current1 = -ina.getCurrent(1);

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
    // BLYNK_LOG("Light Levels: L1=%.0f, L2=%.0f, L3=%.0f, L4=%.0f", L1, L2, L3, L4);
  }
}

void sendBlynk() // send INA3221 data to Blynk every 3 seconds
{
  if (isnan(busV0) || isnan(current0) ||
      isnan(busV1) || isnan(current1) ||
      isnan(busV2) || isnan(current2))
  {
    BLYNK_LOG("Cannot send INA3221 data to Blynk due to NaN values");
    return;
  }
  Blynk.virtualWrite(V0, panAngle);
  Blynk.virtualWrite(V1, tiltAngle);
  Blynk.virtualWrite(V2, current0);
  Blynk.virtualWrite(V3, busV0);
  Blynk.virtualWrite(V4, current1);
  Blynk.virtualWrite(V5, busV1);
  Blynk.virtualWrite(V6, current2);
  Blynk.virtualWrite(V7, busV2);
}

void TaskBlynk(void *pvParameters)
{
  for (;;)
  {

    Blynk.run();
    timer.run();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void TaskUpdateSensors(void *pvParameters)
{
  for (;;)
  {
    // debugRawRFID();
    updateSens();

    // BLYNK_LOG("CH1: Shunt %.3f mV | Bus %.3f V | Current %.3f A", shuntV0, busV0, current0);
    // BLYNK_LOG("CH2: Shunt %.3f mV | Bus %.3f V | Current %.3f A", shuntV1, busV1, current1);
    // BLYNK_LOG("CH3: Shunt %.3f mV | Bus %.3f V | Current %.3f A", shuntV2, busV2, current2);

    // BLYNK_LOG("L1=%.0f | L2=%.0f | L3=%.0f | L4=%.0f | Sum=%.0f | nx=%.3f ny=%.3f | Pan=%.1f Tilt=%.1f | Yaw=%.1f", L1_kalman, L2_kalman, L3_kalman, L4_kalman, sum, nx, ny, panAngle, tiltAngle, currentYaw);

    Serial.printf("L1:%.0f L2:%.0f L3:%.0f L4:%.0f ", L1, L2, L3, L4);                                                                  // for plotting kalman effect in serial plotter
    Serial.printf("L1_filtered:%.0f L2_filtered:%.0f L3_filtered:%.0f L4_filtered:%.0f\n", L1_kalman, L2_kalman, L3_kalman, L4_kalman); // for plotting kalman effect in serial plotter

    vTaskDelay(125 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);

  /*********** INIT I2C ***********/
  Serial.println("Initializing I2C buses...");
  Wire.begin(); // SDA, SCL
  I2CBus2.begin(19, 18, 400000);

  /*********** INIT BH1750 ***********/
  Serial.println("Initializing BH1750 sensors...");
  bh1.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  bh2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire);
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
  ina.begin();
  ina.setAverage(128); // 128 samples for better stability (adjust as needed)
  ina.setMode(0x7127); // default mode: all channels enabled, 4 averages, 1.1ms conversion time

  ina.setShuntR(0, 0.1); // CH1 0.1 ohm (ubah sesuai hardware)
  ina.setShuntR(1, 0.1); // CH2
  ina.setShuntR(2, 0.1); // CH3

  // /*********** MPU6050 ***********/
  // Serial.println("Initializing MPU6050...");
  // delay(100);
  // mpu.initialize();
  // if (!mpu.testConnection())
  // {
  //   Serial.println("MPU6050 NOT CONNECTED!");
  //   // while (1)
  //   //   ;
  // }
  // else
  // {
  //   Serial.println("MPU6050 Connected");
  // }

  delay(1000);

  // simple yaw zeroing
  yaw0 = 0;

  /*********** BLYNK ***********/
  Serial.println("Connecting to WiFi...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // timer.setInterval(20L, updateSens);  // read sensors and update control every 300 ms (adjust as needed)
  timer.setInterval(1000L, readINA);    // read INA3221 every 200 ms
  timer.setInterval(60000L, sendBlynk); // send data to Blynk every 3 second (adjust as needed)
  timer.setInterval(100L, updateServo); // update servo positions every 100 ms (adjust as needed)
  // timer.setInterval(100L, []()
  //                   {
  //                     // BLYNK_LOG("CH1: Shunt %.3f mV | Bus %.3f V | Current %.3f A", shuntV0, busV0, current0);
  //                     // BLYNK_LOG("CH2: Shunt %.3f mV | Bus %.3f V | Current %.3f A", shuntV1, busV1, current1);
  //                     // BLYNK_LOG("CH3: Shunt %.3f mV | Bus %.3f V | Current %.3f A", shuntV2, busV2, current2);

  //                     // BLYNK_LOG("L1=%.0f | L2=%.0f | L3=%.0f | L4=%.0f | Sum=%.0f | nx=%.3f ny=%.3f | Pan=%.1f Tilt=%.1f | Yaw=%.1f", L1, L2, L3, L4, sum, nx, ny, panAngle, tiltAngle, currentYaw);
  //                     Serial.printf("L1:%.0f L2:%.0f L3:%.0f L4:%.0f ", L1, L2, L3, L4);                                                                  // for plotting kalman effect in serial plotter
  //                     Serial.printf("L1_filtered:%.0f L2_filtered:%.0f L3_filtered:%.0f L4_filtered:%.0f\n", L1_kalman, L2_kalman, L3_kalman, L4_kalman); // for plotting kalman effect in serial plotter
  //                   });                                                                                                                                   // log light levels every 1 second for debugging

  xTaskCreatePinnedToCore(TaskBlynk, "blynk", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskUpdateSensors, "sensors", 4096, NULL, 1, NULL, 0);
  // xTaskCreatePinnedToCore(TaskReadIna, "ina", 4096, NULL, 1, NULL, 0);
  // timer.setInterval(1000L, check);
}

void loop()
{
}
