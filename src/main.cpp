#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define LED_PIN 4
#define POT_PIN 34  // Analog Input Pin

Adafruit_MPU6050 mpu;
bool sensorReady = false;

void setup() {
    Serial.begin(115200);
    delay(1000); 

    // Initialize I2C on standard ESP32 DevKit pins
    Wire.begin(21, 22); 

    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU-6050 not detected.");
        sensorReady = false;
    } else {
        Serial.println("[SUCCESS] MPU-6050 connected.");
        sensorReady = true;
        
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    pinMode(LED_PIN, OUTPUT);
    pinMode(POT_PIN, INPUT); // Configure ADC pin
}

void loop() {
    if (!sensorReady) {
        digitalWrite(LED_PIN, HIGH); delay(100);
        digitalWrite(LED_PIN, LOW);  delay(100);
        return; 
    }

    // --- 1. READ POTENTIOMETER & DYNAMICALLY CALCULATE THRESHOLD ---
    // ESP32 ADC reads from 0 to 4095 (12-bit resolution)
    int rawPotValue = analogRead(POT_PIN);
    
    // Convert to a clean 0.0 to 1.0 factor
    float potPercentage = (float)rawPotValue / 4095.0f; 
    
    // Linearly interpolate between 2.0 (low end) and 4.0 (high end)
    float dynamicThreshold = 2.0f + (potPercentage * (4.0f - 2.0f));

    // --- 2. CAPTURE GYRO MEASUREMENTS ---
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    float gx = gyro.gyro.x;
    float gy = gyro.gyro.y;
    float gz = gyro.gyro.z;

    // --- 3. TELEPLOT TELEMETRY STREAM ---
    Serial.printf(">Gyro_X:%.3f\n", gx);
    Serial.printf(">Gyro_Y:%.3f\n", gy);
    Serial.printf(">Gyro_Z:%.3f\n", gz);
    // Plotting the threshold lets you see your limit line move up and down live!
    Serial.printf(">Current_Threshold:%.3f\n", dynamicThreshold);

    // --- 4. ANY-AXIS THRESHOLD DETECTION ---
    if (abs(gx) > dynamicThreshold || abs(gy) > dynamicThreshold || abs(gz) > dynamicThreshold) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println(">LED_Indicator:1");
    } else {
        digitalWrite(LED_PIN, LOW);
        Serial.println(">LED_Indicator:0");
    }

    delay(15); // Smooth telemetry refresh cycle
}