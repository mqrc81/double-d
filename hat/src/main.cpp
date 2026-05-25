#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <esp_now.h>

// ─── Pin definitions ──────────────────────────────────────────────
// Adjust to match your wiring.
// GPIO17/18 are the Heltec OLED I2C pins — safe to reuse
// on the hat board since we're not using the OLED here.
// If you want the OLED for debug output, use different pins
// for the MPU-6050 and update accordingly.
#define SDA_PIN 17
#define SCL_PIN 18

// ─── Sampling ─────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ      50
#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)  // 20ms

// ─── Complementary filter ─────────────────────────────────────────
// MUST match car firmware and calibration firmware exactly.
#define ALPHA 0.96f

// ─── ESP-NOW packet ───────────────────────────────────────────────
// Identical struct definition must exist in car firmware.
// Both sides must agree on layout — packed prevents padding.
typedef struct __attribute__((packed)) {
    float pitch; // degrees
    float roll; // degrees
    int64_t timestamp_us; // microseconds since boot
    uint8_t seq; // sequence number 0-255, wraps around
} HatImuPacket;

// ─── Globals ──────────────────────────────────────────────────────
MPU6050 mpu;
HatImuPacket packet;

float pitch = 0.0f;
float roll = 0.0f;

unsigned long lastSampleTime = 0;
int64_t lastTimerUs = 0;
uint8_t seqNum = 0;

// Broadcast to all ESP-NOW peers on the same channel.
// Car firmware filters by hat MAC so it only processes
// packets from this board.
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ─── ESP-NOW send callback ────────────────────────────────────────
// Called asynchronously after each send attempt.
// Failures at this range should be rare — log and continue.
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[ESPNOW] Send failed");
    }
}

// ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    // ── I2C + MPU6050 ──
    Wire.begin(SDA_PIN, SCL_PIN);
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("[ERROR] MPU6050 not found. Check wiring.");
        while (true) { delay(1000); }
    }
    Serial.println("[OK] MPU6050 connected.");

    // ── Configure MPU6050 ──
    // Default full-scale ranges:
    //   Accelerometer: ±2g  → 16384 LSB/g
    //   Gyroscope:     ±250°/s → 131 LSB/deg/s
    // If you change these via mpu.setFullScaleGyroRange()
    // or mpu.setFullScaleAccelRange(), update the divisors
    // in the main loop accordingly, and do the same in
    // car firmware and calibration firmware.
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    // ── WiFi in STA mode for ESP-NOW ──
    // Not connected to any network — just needed to
    // initialise the radio hardware for ESP-NOW.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Print MAC address — hardcode this into car firmware
    // so the car can filter packets by source.
    Serial.print("[INFO] Hat ESP32 MAC: ");
    Serial.println(WiFi.macAddress());

    // ── ESP-NOW init ──
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] ESP-NOW init failed.");
        while (true) { delay(1000); }
    }
    esp_now_register_send_cb(onDataSent);

    // ── Register broadcast peer ──
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ERROR] Failed to add ESP-NOW peer.");
        while (true) { delay(1000); }
    }

    Serial.printf("[OK] ESP-NOW ready. Sending at %dHz.\n", SAMPLE_RATE_HZ);

    lastSampleTime = millis();
    lastTimerUs = esp_timer_get_time();
}

// ─────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // Pace the loop to SAMPLE_RATE_HZ
    if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
    lastSampleTime = now;

    // ── Read MPU6050 ──
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // ── Compute dt ──
    int64_t nowUs = esp_timer_get_time();
    float dt = (nowUs - lastTimerUs) / 1e6f;
    lastTimerUs = nowUs;

    // Guard against first-loop spike or anomalous dt
    if (dt <= 0.0f || dt > 0.1f) {
        dt = SAMPLE_INTERVAL_MS / 1000.0f;
    }

    // ── Accelerometer angles ──
    // atan2 gives angle in radians — convert to degrees.
    // Assumes MPU6050 is mounted with Z axis pointing up.
    // If your hat mounting orients the IMU differently,
    // you may need to swap axes here. Test by tilting
    // the hat forward and checking pitch increases as expected.
    float accelPitch = atan2f((float) ay, (float) az) * 180.0f / PI;
    float accelRoll = atan2f((float) ax, (float) az) * 180.0f / PI;

    // ── Gyroscope rates (deg/s) ──
    // Divisor matches ±250°/s full-scale range.
    // Update if you changed setFullScaleGyroRange() above:
    //   ±250°/s  → 131.0
    //   ±500°/s  → 65.5
    //   ±1000°/s → 32.8
    //   ±2000°/s → 16.4
    float gyroPitchRate = (float) gx / 131.0f;
    float gyroRollRate = (float) gy / 131.0f;

    // ── Complementary filter ──
    pitch = ALPHA * (pitch + gyroPitchRate * dt) + (1.0f - ALPHA) * accelPitch;
    roll = ALPHA * (roll + gyroRollRate * dt) + (1.0f - ALPHA) * accelRoll;

    // ── Pack and broadcast ──
    packet.pitch = pitch;
    packet.roll = roll;
    packet.timestamp_us = nowUs;
    packet.seq = seqNum++;

    esp_now_send(broadcastMac, (uint8_t *) &packet, sizeof(HatImuPacket));

    // ── Serial debug ──
    // Useful during bench testing to verify filter output.
    // Disable in deployment — 50Hz serial output adds
    // jitter to sample timing and wastes power.
    Serial.printf("[HAT] pitch=%.2f roll=%.2f seq=%d\n",
                  pitch, roll, packet.seq);
}
