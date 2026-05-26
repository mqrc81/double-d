#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>

// ─── Pin definitions ──────────────────────────────────────────────
// Update to match your actual wiring.
// Handoff spec: LED=GPIO2, buzzer=GPIO4, button=GPIO0.
#define SDA_PIN         21
#define SCL_PIN         22
#define LED_PIN         4
#define BUZZER_PIN      33      // TODO adjust to your wiring
#define BUTTON_PIN      0       // Boot button, active LOW

// ─── Sampling ─────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ          20
#define SAMPLE_INTERVAL_US      (1000000 / SAMPLE_RATE_HZ)  // 20000us

// ─── Complementary filter ─────────────────────────────────────────
// Must match hat firmware exactly.
#define ALPHA                   0.96f

// ─── Phase 1 — Alert baseline ─────────────────────────────────────
// How long to collect alert driving data.
// 3 minutes at 20Hz = 3600 samples — sufficient for
// stable mean and std via Welford's algorithm.
#define ALERT_DURATION_MS       (3 * 60 * 1000)

// ─── Phase 2 — Drowsy calibration ─────────────────────────────────
// Number of nod peaks to collect before computing thresholds.
// Passenger presses button at peak of each nod.
#define NOD_COUNT_TARGET        8

// ─── Outlier rejection (Phase 1) ──────────────────────────────────
// Samples exceeding this absolute differential pitch are
// discarded during alert baseline collection — catches sharp
// turns, sudden braking, and hat adjustment mid-drive.
#define OUTLIER_THRESHOLD_DEG   25.0f

// ─── Re-zero duration ─────────────────────────────────────────────
#define REZERO_DURATION_MS      10000   // 10 seconds stationary

// ─── ESP-NOW packet from hat ──────────────────────────────────────
// Must be identical to HatImuPacket in hat firmware.
typedef struct __attribute__((packed)) {
    float pitch;
    float roll;
    int64_t timestamp_us;
    uint8_t seq;
} HatImuPacket;

// ─── System state ─────────────────────────────────────────────────
typedef enum {
    STATE_WAITING, // waiting for first button press
    STATE_REZERO, // 10 second stationary re-zero
    STATE_ALERT_COLLECT, // Phase 1: collecting alert baseline
    STATE_ALERT_DONE, // Phase 1 complete, waiting for Phase 2
    STATE_DROWSY_COLLECT, // Phase 2: collecting nod peaks
    STATE_COMPLETE // all calibration done, saved to NVS
} CalibState;

CalibState state = STATE_WAITING;

// ─── Car IMU ──────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
float carPitch = 0.0f;
float carRoll = 0.0f;
int64_t lastTimerUs = 0;

// ─── Hat data (written by ESP-NOW callback) ───────────────────────
// Access only under hatMux to avoid race conditions on dual-core.
portMUX_TYPE hatMux = portMUX_INITIALIZER_UNLOCKED;
volatile float hatPitch = 0.0f;
volatile float hatRoll = 0.0f;
volatile bool hatDataReady = false;
volatile uint8_t lastHatSeq = 255;

// Hat's MAC address — update this to match your hat ESP32.
// Run the hat firmware once and read MAC from serial output.
uint8_t hatMac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// ─── Re-zero state ────────────────────────────────────────────────
double rezeroSumPitch = 0.0;
double rezeroSumRoll = 0.0;
int rezeroCount = 0;
float rezeroOffsetPitch = 0.0f;
float rezeroOffsetRoll = 0.0f;
unsigned long rezeroStartMs = 0;

// ─── Phase 1: Welford state ───────────────────────────────────────
double wMeanPitch = 0.0, wM2Pitch = 0.0;
double wMeanRoll = 0.0, wM2Roll = 0.0;
int wCount = 0;
unsigned long alertStartMs = 0;

// ─── Phase 2: Nod peak collection ─────────────────────────────────
// Stores (peak differential pitch, drop rate) for each nod.
// Drop rate computed from derivative of differential pitch
// over the 500ms window preceding the button press.
#define MAX_NODS    16
float nodPeaks[MAX_NODS];
float nodRates[MAX_NODS];
int nodCount = 0;

// Rolling buffer for drop rate computation — holds last
// RATE_WINDOW samples of differential pitch.
#define RATE_WINDOW 25          // 25 samples at 20Hz = 1250ms
float pitchHistory[RATE_WINDOW];
int historyIndex = 0;
bool historyFull = false;

// ─── NVS ──────────────────────────────────────────────────────────
Preferences prefs;

// ─── Forward declarations ─────────────────────────────────────────
void updateCarIMU();

float getDifferentialPitch();

float getDifferentialRoll();

float computeDropRate();

void updatePitchHistory(float diffPitch);

void handleButton();

void onButtonPressed();

void runRezero();

void runAlertCollect();

void runDrowsyCollect();

void saveToNVS();

void buzzerBeeps(int count, int durationMs = 150, int gapMs = 150);

void setLED(bool on);

void onDataReceived(const uint8_t *mac, const uint8_t *data, int len);

// ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    setLED(false);
    digitalWrite(BUZZER_PIN, LOW);

    // ── Car IMU ──
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!mpu.begin()) {
        Serial.println("[ERROR] Car MPU6050 not found. Check wiring.");
        while (true) { delay(1000); }
    }
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[OK] Car MPU6050 connected.");

    // ── WiFi for ESP-NOW ──
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.print("[INFO] Car MAC: ");
    Serial.println(WiFi.macAddress());

    // ── ESP-NOW ──
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] ESP-NOW init failed.");
        while (true) { delay(1000); }
    }
    esp_now_register_recv_cb(onDataReceived);

    // Register hat as peer so we can filter packets by source MAC.
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, hatMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[WARN] Could not register hat peer.");
        // Non-fatal — MAC filtering in the callback still works.
    }

    memset(pitchHistory, 0, sizeof(pitchHistory));
    lastTimerUs = esp_timer_get_time();

    Serial.println("[OK] Ready. Press Boot button to begin calibration.");
    Serial.println("     Phase 1: drive normally for 3 minutes.");
    Serial.println("     Phase 2: simulate drowsy nods (passenger presses button at each peak).");
}

// ─────────────────────────────────────────────────────────────────
void loop() {
    static int64_t lastSampleUs = 0;
    static unsigned long lastBlinkMs = 0;
    static bool ledBlinkState = false;

    int64_t nowUs = esp_timer_get_time();

    // Pace main loop to sample rate — all timing from one clock
    if (nowUs - lastSampleUs < SAMPLE_INTERVAL_US) return;
    lastSampleUs = nowUs;

    updateCarIMU();
    handleButton();

    unsigned long nowMs = millis();
    switch (state) {
        case STATE_WAITING:
            if (nowMs - lastBlinkMs > 1000) {
                ledBlinkState = !ledBlinkState;
                setLED(ledBlinkState);
                lastBlinkMs = nowMs;
            }
            break;

        case STATE_REZERO:
            runRezero();
            if (nowMs - lastBlinkMs > 200) {
                ledBlinkState = !ledBlinkState;
                setLED(ledBlinkState);
                lastBlinkMs = nowMs;
            }
            break;

        case STATE_ALERT_COLLECT:
            runAlertCollect();
            if (nowMs - lastBlinkMs > 100) {
                ledBlinkState = !ledBlinkState;
                setLED(ledBlinkState);
                lastBlinkMs = nowMs;
            }
            break;

        case STATE_ALERT_DONE:
            setLED(true);
            break;

        case STATE_DROWSY_COLLECT:
            runDrowsyCollect();
            if (nowMs - lastBlinkMs > 300) {
                ledBlinkState = !ledBlinkState;
                setLED(ledBlinkState);
                lastBlinkMs = nowMs;
            }
            break;

        case STATE_COMPLETE:
            setLED(false);
            break;
    }
}

// ─────────────────────────────────────────────────────────────────
// ESP-NOW receive callback.
// Called on a FreeRTOS task — keep it short.
// Write data under critical section; main loop reads under the same.
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (memcmp(mac, hatMac, 6) != 0) return;
    if (len != sizeof(HatImuPacket)) return;

    HatImuPacket pkt;
    memcpy(&pkt, data, sizeof(HatImuPacket));

    uint8_t expected = lastHatSeq + 1;
    if (lastHatSeq != 255 && pkt.seq != expected) {
        Serial.printf("[WARN] Dropped hat packet(s): expected %d got %d\n",
                      expected, pkt.seq);
    }

    taskENTER_CRITICAL(&hatMux);
    hatPitch = pkt.pitch;
    hatRoll = pkt.roll;
    lastHatSeq = pkt.seq;
    hatDataReady = true;
    taskEXIT_CRITICAL(&hatMux);
}

// ─────────────────────────────────────────────────────────────────
void updateCarIMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int64_t nowUs = esp_timer_get_time();
    float dt = (nowUs - lastTimerUs) / 1e6f;
    lastTimerUs = nowUs;
    if (dt <= 0.0f || dt > 0.1f) dt = SAMPLE_INTERVAL_US / 1e6f;

    // Adafruit library: acceleration in m/s², gyro in rad/s
    float accelPitch = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
    float accelRoll = atan2f(a.acceleration.x, a.acceleration.z) * 180.0f / PI;
    float gyroPitchRate = g.gyro.x * 180.0f / PI; // rad/s → deg/s
    float gyroRollRate = g.gyro.y * 180.0f / PI;

    carPitch = ALPHA * (carPitch + gyroPitchRate * dt) + (1.0f - ALPHA) * accelPitch;
    carRoll = ALPHA * (carRoll + gyroRollRate * dt) + (1.0f - ALPHA) * accelRoll;
}

// ─────────────────────────────────────────────────────────────────
// Differential signal: hat minus car, minus re-zero offset.
// This is the cleaned head movement signal used for all
// calibration and detection logic.
float getDifferentialPitch() {
    return hatPitch - carPitch - rezeroOffsetPitch;
}

float getDifferentialRoll() {
    return hatRoll - carRoll - rezeroOffsetRoll;
}

// ─────────────────────────────────────────────────────────────────
void updatePitchHistory(float diffPitch) {
    pitchHistory[historyIndex] = diffPitch;
    historyIndex = (historyIndex + 1) % RATE_WINDOW;
    if (historyIndex == 0) historyFull = true;
}

// ─────────────────────────────────────────────────────────────────
// Computes drop rate (deg/s) over the last RATE_WINDOW samples.
// Uses endpoint slope — sufficient for characterising a slow nod.
// Positive = head moving forward (dropping).
float computeDropRate() {
    int n = historyFull ? RATE_WINDOW : historyIndex;
    if (n < 2) return 0.0f;

    int firstIdx = historyFull ? historyIndex : 0;
    float first = pitchHistory[firstIdx];
    float last = pitchHistory[(historyIndex - 1 + RATE_WINDOW) % RATE_WINDOW];
    float timeWindow = (float) (n - 1) / (float) SAMPLE_RATE_HZ;

    return (last - first) / timeWindow;
}

// ─────────────────────────────────────────────────────────────────
// Button debounce and falling-edge detection.
// Restarts debounce timer on any raw state change; only acts once
// the signal has been stable for 50ms.
void handleButton() {
    static bool lastRawState = HIGH;
    static bool wasPressed = false;
    static unsigned long lastDebounce = 0;

    bool raw = digitalRead(BUTTON_PIN);

    if (raw != lastRawState) {
        lastDebounce = millis();
        lastRawState = raw;
    }

    if (millis() - lastDebounce < 50) return;

    if (raw == LOW && !wasPressed) {
        wasPressed = true;
        onButtonPressed();
    } else if (raw == HIGH) {
        wasPressed = false;
    }
}

// ─────────────────────────────────────────────────────────────────
void onButtonPressed() {
    switch (state) {
        case STATE_WAITING:
            Serial.println("[CAL] Starting re-zero. Sit still for 10 seconds.");
            rezeroSumPitch = 0.0;
            rezeroSumRoll = 0.0;
            rezeroCount = 0;
            rezeroStartMs = millis();
            state = STATE_REZERO;
            break;

        case STATE_ALERT_DONE:
            Serial.println("[CAL] Phase 2: simulate drowsy nods.");
            Serial.println("      Passenger presses button at peak of each nod.");
            Serial.printf("      Need %d nods.\n", NOD_COUNT_TARGET);
            nodCount = 0;
            historyIndex = 0;
            historyFull = false;
            memset(pitchHistory, 0, sizeof(pitchHistory));
            state = STATE_DROWSY_COLLECT;
            break;

        case STATE_DROWSY_COLLECT: {
            // Snapshot hat data under critical section
            float localHatPitch;
            bool gotData = false;
            taskENTER_CRITICAL(&hatMux);
            if (hatDataReady) {
                localHatPitch = hatPitch;
                gotData = true;
            }
            taskEXIT_CRITICAL(&hatMux);

            if (!gotData) {
                Serial.println("[WARN] No hat data yet — is hat powered on?");
                return;
            }

            float diffPitch = localHatPitch - carPitch - rezeroOffsetPitch;
            float dropRate = computeDropRate();

            if (nodCount < MAX_NODS) {
                nodPeaks[nodCount] = diffPitch;
                nodRates[nodCount] = dropRate;
                nodCount++;

                Serial.printf("[CAL] Nod %d/%d — peak=%.2f deg, rate=%.2f deg/s\n",
                              nodCount, NOD_COUNT_TARGET, diffPitch, dropRate);
                buzzerBeeps(1, 80, 0);

                if (nodCount >= NOD_COUNT_TARGET) {
                    Serial.println("[CAL] Nod collection complete. Saving...");
                    saveToNVS();
                    buzzerBeeps(3);
                    state = STATE_COMPLETE;
                    Serial.println("[CAL] Calibration complete. Flash DD firmware.");
                }
            }
            break;
        }

        default:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────
void runRezero() {
    float localHatPitch, localHatRoll;
    bool gotData = false;
    taskENTER_CRITICAL(&hatMux);
    if (hatDataReady) {
        localHatPitch = hatPitch;
        localHatRoll = hatRoll;
        hatDataReady = false;
        gotData = true;
    }
    taskEXIT_CRITICAL(&hatMux);
    if (!gotData) return;

    // Accumulate raw differential — no offset applied yet
    rezeroSumPitch += localHatPitch - carPitch;
    rezeroSumRoll += localHatRoll - carRoll;
    rezeroCount++;

    if (millis() - rezeroStartMs >= REZERO_DURATION_MS) {
        rezeroOffsetPitch = (float) (rezeroSumPitch / rezeroCount);
        rezeroOffsetRoll = (float) (rezeroSumRoll / rezeroCount);

        Serial.printf("[CAL] Re-zero complete. Offset pitch=%.3f roll=%.3f\n",
                      rezeroOffsetPitch, rezeroOffsetRoll);

        wMeanPitch = wM2Pitch = 0.0;
        wMeanRoll = wM2Roll = 0.0;
        wCount = 0;
        alertStartMs = millis();

        Serial.println("[CAL] Phase 1: drive normally for 3 minutes.");
        buzzerBeeps(1);
        state = STATE_ALERT_COLLECT;
    }
}

// ─────────────────────────────────────────────────────────────────
void runAlertCollect() {
    float localHatPitch, localHatRoll;
    bool gotData = false;
    taskENTER_CRITICAL(&hatMux);
    if (hatDataReady) {
        localHatPitch = hatPitch;
        localHatRoll = hatRoll;
        hatDataReady = false;
        gotData = true;
    }
    taskEXIT_CRITICAL(&hatMux);
    if (!gotData) return;

    float diffPitch = localHatPitch - carPitch - rezeroOffsetPitch;
    float diffRoll = localHatRoll - carRoll - rezeroOffsetRoll;

    updatePitchHistory(diffPitch);

    if (fabsf(diffPitch) > OUTLIER_THRESHOLD_DEG) {
        Serial.printf("[CAL] Outlier rejected: %.2f deg\n", diffPitch);
        return;
    }

    // Welford's online algorithm — pitch
    wCount++;
    double deltaPitch = diffPitch - wMeanPitch;
    wMeanPitch += deltaPitch / wCount;
    wM2Pitch += deltaPitch * (diffPitch - wMeanPitch);

    // Welford's online algorithm — roll
    double deltaRoll = diffRoll - wMeanRoll;
    wMeanRoll += deltaRoll / wCount;
    wM2Roll += deltaRoll * (diffRoll - wMeanRoll);

    // Progress log every 30 seconds
    static unsigned long lastLogMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastLogMs >= 30000) {
        lastLogMs = nowMs;
        unsigned long elapsed = nowMs - alertStartMs;
        Serial.printf("[CAL] Phase 1: %lu / %d seconds (%d samples)\n",
                      elapsed / 1000, ALERT_DURATION_MS / 1000, wCount);
    }

    if (millis() - alertStartMs >= ALERT_DURATION_MS) {
        float stdPitch = (wCount > 1) ? sqrtf((float) (wM2Pitch / (wCount - 1))) : 0.0f;
        float stdRoll = (wCount > 1) ? sqrtf((float) (wM2Roll / (wCount - 1))) : 0.0f;

        Serial.println("[CAL] Phase 1 complete.");
        Serial.printf("  mean_alert_pitch = %.3f deg\n", (float) wMeanPitch);
        Serial.printf("  std_alert_pitch  = %.3f deg\n", stdPitch);
        Serial.printf("  mean_alert_roll  = %.3f deg\n", (float) wMeanRoll);
        Serial.printf("  std_alert_roll   = %.3f deg\n", stdRoll);
        Serial.println("  Press button to begin Phase 2 (drowsy nods).");

        prefs.begin("dd_profile", false);
        prefs.putFloat("mean_alert_pitch", (float) wMeanPitch);
        prefs.putFloat("std_alert_pitch", stdPitch);
        prefs.putFloat("mean_alert_roll", (float) wMeanRoll);
        prefs.putFloat("std_alert_roll", stdRoll);
        prefs.putBool("phase1_done", true);
        prefs.end();

        buzzerBeeps(2);
        state = STATE_ALERT_DONE;
    }
}

// ─────────────────────────────────────────────────────────────────
void runDrowsyCollect() {
    float localHatPitch;
    bool gotData = false;
    taskENTER_CRITICAL(&hatMux);
    if (hatDataReady) {
        localHatPitch = hatPitch;
        hatDataReady = false;
        gotData = true;
    }
    taskEXIT_CRITICAL(&hatMux);
    if (!gotData) return;

    float diffPitch = localHatPitch - carPitch - rezeroOffsetPitch;
    updatePitchHistory(diffPitch);

    static unsigned long lastPrintMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastPrintMs > 200) {
        lastPrintMs = nowMs;
        Serial.printf("[LIVE] diff_pitch=%.2f  nods=%d/%d\n",
                      diffPitch, nodCount, NOD_COUNT_TARGET);
    }
}

// ─────────────────────────────────────────────────────────────────
void saveToNVS() {
    float sumPeaks = 0.0f, sumRates = 0.0f;
    for (int i = 0; i < nodCount; i++) {
        sumPeaks += nodPeaks[i];
        sumRates += fabsf(nodRates[i]);
    }
    float meanDrowsyPitch = sumPeaks / nodCount;
    float meanDrowsyRate = sumRates / nodCount;

    Serial.println("[CAL] Drowsy calibration summary:");
    Serial.printf("  mean_drowsy_pitch = %.3f deg\n", meanDrowsyPitch);
    Serial.printf("  mean_drowsy_rate  = %.3f deg/s\n", meanDrowsyRate);
    Serial.println("  Individual nods:");
    for (int i = 0; i < nodCount; i++) {
        Serial.printf("    [%d] peak=%.2f deg  rate=%.2f deg/s\n",
                      i, nodPeaks[i], fabsf(nodRates[i]));
    }

    prefs.begin("dd_profile", false);
    prefs.putFloat("mean_drowsy_pitch", meanDrowsyPitch);
    prefs.putFloat("mean_drowsy_rate", meanDrowsyRate);
    prefs.putInt("nod_count", nodCount);
    prefs.putBool("phase2_done", true);
    prefs.end();

    Serial.println("[CAL] All values saved to NVS namespace 'dd_profile'.");
    // Serial.println("      Keys: mean_alert_pitch, std_alert_pitch,");
    // Serial.println("             mean_alert_roll,  std_alert_roll,");
    // Serial.println("             mean_drowsy_pitch, mean_drowsy_rate");
}

// ─────────────────────────────────────────────────────────────────
void buzzerBeeps(int count, int durationMs, int gapMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(durationMs);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < count - 1) delay(gapMs);
    }
}

void setLED(bool on) {
    digitalWrite(LED_PIN, on ? HIGH : LOW);
}
