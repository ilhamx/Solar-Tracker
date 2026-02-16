/*********** BLYNK ***********/

#include <Arduino.h>

#include <MPU6050.h>
MPU6050 mpu;

float yaw0 = 0;
float currentYaw = 0;

#define YAW_LIMIT 160   // max rotation from start (degrees)


// #define BLYNK_TEMPLATE_ID "TMPLxxxx"
// #define BLYNK_TEMPLATE_NAME "Solar Tracker"
// #define BLYNK_AUTH_TOKEN "YOUR_AUTH_TOKEN"

#define BLYNK_TEMPLATE_ID "TMPL6JETrKguB"
#define BLYNK_TEMPLATE_NAME "Percobaan IoT"
#define BLYNK_AUTH_TOKEN "4AIRXorIi0sIywRlka-ZszcuIWZoyMKC"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "URT";
char pass[] = "unimusrobotic";

BlynkTimer timer; // Create a timer object to handle timed events

/*********** I2C ***********/
#include <Wire.h>
TwoWire I2CBus1 = TwoWire(0);  // default
TwoWire I2CBus2 = TwoWire(1);  // second bus

/*********** BH1750 ***********/
#include <BH1750.h>

BH1750 bh1;  // Bus1 addr 0x23
BH1750 bh2;  // Bus1 addr 0x5C
BH1750 bh3;  // Bus2 addr 0x23
BH1750 bh4;  // Bus2 addr 0x5C

/*********** OLED (Bus1) ***********/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2CBus1, -1);

/*********** SERVO ***********/
#include <ESP32Servo.h>
Servo servoPan;
Servo servoTilt;

#define PAN_PIN  26
#define TILT_PIN 27

float panAngle  = 90;
float tiltAngle = 90;

#define PAN_MIN   50
#define PAN_MAX  140
#define TILT_MIN  55
#define TILT_MAX 125

/*********** TRACKER PARAM ***********/
float Kp_X = 5.0;
float Kp_Y = 1.0;
float deadzone = 0.02;
unsigned long lastUpdate = 0;

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

  if (abs(nx) < deadzone) nx = 0;
  if (abs(ny) < deadzone) ny = 0;

  /*********** CONTROL ***********/
  panAngle  += nx * Kp_X; // multiplied by 10 to increase sensitivity
  tiltAngle += ny * Kp_Y;

  panAngle  = constrain(panAngle,  PAN_MIN,  PAN_MAX);
  tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);

  servoPan.write(panAngle); // offset for calibration
  servoTilt.write(tiltAngle);

  /*********** OLED ***********/
  // Serial.println("writing to lcd");
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("Lux: %.0f\n", sum);
  display.printf("nx: %.3f\n", nx);
  display.printf("ny: %.3f\n", ny);
  display.printf("Pan: %.1f\n", panAngle);
  display.printf("Tilt: %.1f\n", tiltAngle);
  display.display();

  /*********** BLYNK ***********/
  // Serial.println("sending data to blynk.");
  // Blynk.virtualWrite(V0, sum);
  // Blynk.virtualWrite(V1, nx);
  // Blynk.virtualWrite(V2, ny);
  // Blynk.virtualWrite(V3, panAngle);
  // Blynk.virtualWrite(V4, tiltAngle);

  BLYNK_LOG("L1Lux %.0f | L2Lux %.0f | L3Lux %.0f | L4Lux %.0f | Sum %.0f | nx %.3f ny %.3f | Pan %.1f Tilt %.1f", L1, L2, L3, L4, sum, nx, ny, panAngle, tiltAngle);

}

void setup() {
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

  /*********** OLED ***********/
  Serial.println("Initializing OLED display...");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true); // address 0x3C, reset pin not used
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Solar Tracker");
  display.display();

  /*********** SERVO ***********/
  Serial.println("Initializing servos...");
  servoPan.attach(PAN_PIN);
  servoTilt.attach(TILT_PIN);
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  /*********** BLYNK ***********/
  Serial.println("Connecting to WiFi...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(200L, myTimerEvent);
  // timer.setInterval(1000L, check);

}

void loop() {
  Blynk.run();
  timer.run();

  // if (millis() - lastUpdate < 500) return; // 2 Hz
  // lastUpdate = millis();
}
