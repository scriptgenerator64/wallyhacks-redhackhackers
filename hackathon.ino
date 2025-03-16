#include <Wire.h>
#include <MPU6500_WE.h>

#define MPU6500_ADDR 0x68
#define TRIG_PIN 9
#define ECHO_PIN 10
#define PHOTO_PIN A2
#define BEEPER 11

const int distanceThreshold = 30;
const int photoThreshold = 30;
const long flickerStrikeInterval = 2500;
const long ultrasonicInterval = 250;
const int flickerStrikeThreshold = 3;
const float gyrThreshold = 30.0;

int prevValue = -10000;
unsigned long previousMillis = 0;
int strike = 0;
bool objectDetected = false;
MPU6500_WE myMPU6500(MPU6500_ADDR);
xyzFloat prevGyr = {0, 0, 0};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BEEPER, OUTPUT);

  if (!myMPU6500.init()) {
    Serial.println("MPU6500 not found!");
    while (1);
  }
  myMPU6500.autoOffsets();
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  pinMode(PHOTO_PIN, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  // Reset flicker strikes
  if (currentMillis - previousMillis >= flickerStrikeInterval) {
    previousMillis = currentMillis;
    strike = 0;
  }

  // Check distance
  if (currentMillis - previousMillis >= ultrasonicInterval) {
    previousMillis = currentMillis;
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    float distance = duration * 0.0343 / 2;

    if (distance > 0 && distance <= distanceThreshold) {
      if (!objectDetected) {
        Serial.println("OBJECT DETECTED!");
        Serial.print("Distance (cm): ");
        Serial.println(distance);
        objectDetected = true;
        tone(BEEPER, 660, 150); // E5
        delay(150);
        tone(BEEPER, 660, 150); // E5
        delay(300);
        tone(BEEPER, 660, 150); // E5
        delay(300);
        tone(BEEPER, 510, 150); // C5
        delay(150);
        tone(BEEPER, 660, 250); // E5
        delay(300);
        tone(BEEPER, 770, 150); // G5
        delay(550);
        tone(BEEPER, 380, 150); // G4
        delay(575);
      }
    } else {
      objectDetected = false;
    }
  }

  // Photoresistor flicker detection
  int sensorValue = analogRead(PHOTO_PIN);
  if (prevValue < 0) {
    prevValue = sensorValue;
  }
  if (abs(prevValue - sensorValue) > photoThreshold) {
    strike++;
    prevValue = sensorValue;
  }
  if (strike >= flickerStrikeThreshold) {
    Serial.println("FLICKERING!");
    Serial.print("Photoresistor: ");
    Serial.println(sensorValue);
    strike = 0;
  }

  // Gyroscope bump detection
  xyzFloat gyr = myMPU6500.getGyrValues();
  if ((fabs(gyr.x - prevGyr.x) + fabs(gyr.y - prevGyr.y) + fabs(gyr.z - prevGyr.z)) >= gyrThreshold) {
    Serial.print("BUMP DETECTED!");
    Serial.println("Gyro(dps): ");
    Serial.print(gyr.x); Serial.print(", ");
    Serial.print(gyr.y); Serial.print(", ");
    Serial.println(gyr.z);
  }
  prevGyr = gyr;
  delay(100);
}
