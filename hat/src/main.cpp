#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>

// ─── Pin definitions ──────────────────────────────────────────────
// Heltec OLED I2C pins — safe to reuse on the hat board since we're
// not using the OLED here. If you want the OLED for debug output,
// use different pins for the MPU-6050 and update accordingly.
#define SDA_PIN 17
#define SCL_PIN 18

// ─── Sampling ─────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ         50
#define SAMPLE_INTERVAL_US     (1000000 / SAMPLE_RATE_HZ)  // 20000us

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
Adafruit_MPU6050 mpu;
HatImuPacket packet;

float pitch = 0.0f;
float roll = 0.0f;

// Both sample gating and dt computation use esp_timer_get_time()
// so they share a single clock — no millis/microseconds mismatch.
int64_t lastSampleUs = 0;
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

    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 not found. Check wiring.");
        while (true) { delay(1000); }
    }
    Serial.println("[OK] MPU6050 connected.");

    // ── Configure MPU6050 ──
    // ±250°/s gyro and ±2g accel match the handoff spec.
    // If you change these here, update the same settings in car
    // firmware and calibration firmware — the Adafruit library
    // handles unit conversion internally so the complementary
    // filter math stays the same, but ALPHA may need retuning.
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // ── WiFi in STA mode for ESP-NOW ──
    // Not connected to any network — just needed to initialise
    // the radio hardware for ESP-NOW.
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

    // Seed both timers from the same clock read so the first
    // dt computation is correct and the first sample fires
    // immediately.
    int64_t now = esp_timer_get_time();
    lastSampleUs = now;
    lastTimerUs = now;
}

// ─────────────────────────────────────────────────────────────────
void loop() {
    int64_t nowUs = esp_timer_get_time();

    // Pace the loop to SAMPLE_RATE_HZ using the same clock
    // as dt — no millis/esp_timer mismatch.
    if (nowUs - lastSampleUs < SAMPLE_INTERVAL_US) return;
    lastSampleUs = nowUs;

    // ── Read MPU6050 ──
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // ── Compute dt ──
    float dt = (nowUs - lastTimerUs) / 1e6f;
    lastTimerUs = nowUs;

    // Guard against first-loop spike or anomalous dt.
    if (dt <= 0.0f || dt > 0.1f) {
        dt = SAMPLE_INTERVAL_US / 1e6f;
    }

    // ── Accelerometer angles ──
    // atan2 gives angle in radians — convert to degrees.
    // Assumes MPU6050 is mounted with Z axis pointing up.
    // If the hat mounting orients the IMU differently,
    // swap axes here and do the same in car/calibration firmware.
    // Test by tilting the hat forward and verifying pitch increases.
    float accelPitch = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
    float accelRoll = atan2f(a.acceleration.x, a.acceleration.z) * 180.0f / PI;

    // ── Gyroscope rates (deg/s) ──
    // Adafruit library outputs rad/s — convert to deg/s for the filter.
    float gyroPitchRate = g.gyro.x * 180.0f / PI;
    float gyroRollRate = g.gyro.y * 180.0f / PI;

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
    // Disable before deployment — 50Hz serial output adds jitter
    // to sample timing and wastes power.
    Serial.printf("[HAT] pitch=%.2f roll=%.2f seq=%d\n",
                  pitch, roll, packet.seq);
}
