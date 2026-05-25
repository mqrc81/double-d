#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>

// ─── Pin definitions ──────────────────────────────────────────────
#define SDA_PIN         17
#define SCL_PIN         18
#define LED_PIN         35
#define BUZZER_PIN      33      // adjust to your wiring
#define BUTTON_PIN      0       // PRG button, active LOW

// ─── Sampling ─────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ          50
#define SAMPLE_INTERVAL_MS      (1000 / SAMPLE_RATE_HZ)   // 20ms

// ─── Complementary filter ─────────────────────────────────────────
// Must match hat firmware exactly.
#define ALPHA                   0.96f

// ─── Phase 1 — Alert baseline ─────────────────────────────────────
// How long to collect alert driving data.
// 3 minutes at 50Hz = 9000 samples — sufficient for
// stable mean and std via Welford's algorithm.
#define ALERT_DURATION_MS       (3 * 60 * 1000)

// ─── Phase 2 — Drowsy calibration ────────────────────────────────
// Number of nod peaks to collect before computing thresholds.
// Passenger presses button at peak of each nod.
#define NOD_COUNT_TARGET        8

// ─── Outlier rejection (Phase 1) ─────────────────────────────────
// Samples exceeding this absolute differential pitch are
// discarded during alert baseline collection — catches sharp
// turns, sudden braking, and hat adjustment mid-drive.
// Generous threshold so normal driving is never rejected.
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
MPU6050 mpu;
float carPitch = 0.0f;
float carRoll = 0.0f;
int64_t lastTimerUs = 0;

// ─── Hat data (written by ESP-NOW callback) ───────────────────────
volatile float hatPitch = 0.0f;
volatile float hatRoll = 0.0f;
volatile bool hatDataReady = false;
volatile uint8_t lastHatSeq = 255;

// Hat's MAC address — update this to match your hat ESP32.
// Run the hat firmware once and read MAC from serial output.
uint8_t hatMac[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// ─── Re-zero state ────────────────────────────────────────────────
// Accumulates differential signal during stationary re-zero
// to compute placement offset for this session.
double rezeroSumPitch = 0.0;
double rezeroSumRoll = 0.0;
int rezeroCount = 0;
float rezeroOffsetPitch = 0.0f; // applied to all subsequent readings
float rezeroOffsetRoll = 0.0f;
unsigned long rezeroStartMs = 0;

// ─── Phase 1: Welford state ───────────────────────────────────────
double wMeanPitch = 0.0, wM2Pitch = 0.0;
double wMeanRoll = 0.0, wM2Roll = 0.0;
int wCount = 0;
unsigned long alertStartMs = 0;

// ─── Phase 2: Nod peak collection ────────────────────────────────
// Stores (peak differential pitch, drop rate) for each nod.
// Drop rate computed from derivative of differential pitch
// over the 500ms window preceding the button press.
#define MAX_NODS    16
float nodPeaks[MAX_NODS]; // peak differential pitch at each nod
float nodRates[MAX_NODS]; // drop rate (deg/s) leading into peak
int nodCount = 0;

// Rolling buffer for derivative computation — holds last
// RATE_WINDOW samples of differential pitch.
#define RATE_WINDOW 25          // 25 samples at 50Hz = 500ms
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

void runRezero();

void runAlertCollect();

void runDrowsyCollect();

void saveToNVS();

void buzzerBeeps(int count, int durationMs = 150, int gapMs = 150);

void setLED(bool on);

void blinkLED(int periodMs);

void printState();

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
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("[ERROR] Car MPU6050 not found.");
        while (true) { delay(1000); }
    }
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
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

    // Register hat as peer (for filtering — we only process
    // packets from the known hat MAC, ignoring any other
    // ESP-NOW devices on the same channel)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, hatMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[WARN] Could not register hat peer.");
        // Non-fatal — we still filter by MAC in the callback
    }

    // Initialise pitch history buffer to zero
    memset(pitchHistory, 0, sizeof(pitchHistory));

    lastTimerUs = esp_timer_get_time();

    Serial.println("[OK] Ready. Press PRG button to begin calibration.");
    Serial.println("     Phase 1: drive normally for 3 minutes.");
    Serial.println("     Phase 2: simulate drowsy nods (passenger presses button at each peak).");
}

// ─────────────────────────────────────────────────────────────────
void loop() {
    static unsigned long lastSampleMs = 0;
    static unsigned long lastBlinkMs = 0;
    static bool ledBlinkState = false;

    unsigned long now = millis();

    // ── Pace main loop to sample rate ──
    if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
    lastSampleMs = now;

    // ── Always update car IMU ──
    updateCarIMU();

    // ── Handle button ──
    handleButton();

    // ── State-specific logic ──
    switch (state) {
        case STATE_WAITING:
            // Slow blink — waiting for user
            if (now - lastBlinkMs > 1000) {
                ledBlinkState = !ledBlinkState;
                setLED(ledBlinkState);
                lastBlinkMs = now;
            }
            break;

        case STATE_REZERO:
            runRezero();
            // Fast blink during re-zero
            if (now - lastBlinkMs > 200) {
                ledBlinkState = !ledBlinkState;
                setLED(ledBlinkState);
                lastBlinkMs = now;
            }
            break;

        case STATE_ALERT_COLLECT:
            runAlertCollect();
            // Very fast blink during alert collection
            if (now - lastBlinkMs > 100) {
                ledBlinkState = !ledBlinkState;
                setLED(ledBlinkState);
                lastBlinkMs = now;
            }
            break;

        case STATE_ALERT_DONE:
            // Solid LED — waiting for Phase 2 to begin
            setLED(true);
            break;

        case STATE_DROWSY_COLLECT:
            runDrowsyCollect();
            // Double blink pattern — distinct from Phase 1
            if (now - lastBlinkMs > 300) {
                ledBlinkState = !ledBlinkState;
                setLED(ledBlinkState);
                lastBlinkMs = now;
            }
            break;

        case STATE_COMPLETE:
            // Solid LED off — done
            setLED(false);
            break;
    }
}

// ─────────────────────────────────────────────────────────────────
// ESP-NOW receive callback
// Called on a separate FreeRTOS task — keep it short.
// Only copy data and set flag; processing happens in main loop.
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    // Filter by hat MAC — ignore any other ESP-NOW devices
    if (memcmp(mac, hatMac, 6) != 0) return;
    if (len != sizeof(HatImuPacket)) return;

    HatImuPacket pkt;
    memcpy(&pkt, data, sizeof(HatImuPacket));

    // Check for dropped packets via sequence number
    uint8_t expected = lastHatSeq + 1;
    if (lastHatSeq != 255 && pkt.seq != expected) {
        Serial.printf("[WARN] Dropped hat packet(s): expected %d got %d\n",
                      expected, pkt.seq);
    }
    lastHatSeq = pkt.seq;

    hatPitch = pkt.pitch;
    hatRoll = pkt.roll;
    hatDataReady = true;
}

// ─────────────────────────────────────────────────────────────────
void updateCarIMU() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int64_t nowUs = esp_timer_get_time();
    float dt = (nowUs - lastTimerUs) / 1e6f;
    lastTimerUs = nowUs;

    if (dt <= 0.0f || dt > 0.1f) dt = SAMPLE_INTERVAL_MS / 1000.0f;

    float accelPitch = atan2f((float) ay, (float) az) * 180.0f / PI;
    float accelRoll = atan2f((float) ax, (float) az) * 180.0f / PI;

    float gyroPitchRate = (float) gx / 131.0f;
    float gyroRollRate = (float) gy / 131.0f;

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
// Maintains a rolling buffer of recent differential pitch values.
// Called every sample during active collection phases.
void updatePitchHistory(float diffPitch) {
    pitchHistory[historyIndex] = diffPitch;
    historyIndex = (historyIndex + 1) % RATE_WINDOW;
    if (historyIndex == 0) historyFull = true;
}

// ─────────────────────────────────────────────────────────────────
// Computes drop rate (deg/s) over the last RATE_WINDOW samples.
// Uses simple linear regression slope on the history buffer.
// Positive value = head moving forward (dropping).
// Called at button press during Phase 2 to characterise
// the drop leading into each nod peak.
float computeDropRate() {
    int n = historyFull ? RATE_WINDOW : historyIndex;
    if (n < 2) return 0.0f;

    // Simple slope: (last value - first value) / time_window
    // Robust enough for 500ms window at 50Hz.
    // Could be replaced with proper linear regression if needed.
    int firstIdx = historyFull ? historyIndex : 0;
    float first = pitchHistory[firstIdx];
    float last = pitchHistory[(historyIndex - 1 + RATE_WINDOW) % RATE_WINDOW];
    float timeWindow = (float) (n - 1) / (float) SAMPLE_RATE_HZ;

    return (last - first) / timeWindow; // deg/s, positive = forward drop
}

// ─────────────────────────────────────────────────────────────────
void handleButton() {
    static bool lastButtonState = HIGH;
    static unsigned long lastDebounce = 0;
    bool currentState = digitalRead(BUTTON_PIN);

    // Debounce
    if (currentState != lastButtonState) {
        lastDebounce = millis();
    }
    if (millis() - lastDebounce < 50) {
        lastButtonState = currentState;
        return;
    }
    lastButtonState = currentState;

    // Only act on falling edge (press, not release)
    static bool wasPressed = false;
    if (currentState == LOW && !wasPressed) {
        wasPressed = true;
        onButtonPressed();
    } else if (currentState == HIGH) {
        wasPressed = false;
    }
}

// ─────────────────────────────────────────────────────────────────
void onButtonPressed() {
    switch (state) {
        case STATE_WAITING:
            // First press — begin re-zero
            Serial.println("[CAL] Starting re-zero. Sit still for 10 seconds.");
            rezeroSumPitch = 0.0;
            rezeroSumRoll = 0.0;
            rezeroCount = 0;
            rezeroStartMs = millis();
            state = STATE_REZERO;
            break;

        case STATE_ALERT_DONE:
            // Press after Phase 1 complete — begin Phase 2
            Serial.println("[CAL] Phase 2: simulate drowsy nods.");
            Serial.println("      Passenger presses button at peak of each nod.");
            Serial.printf("      Need %d nods.\n", NOD_COUNT_TARGET);
            nodCount = 0;
            historyIndex = 0;
            historyFull = false;
            memset(pitchHistory, 0, sizeof(pitchHistory));
            state = STATE_DROWSY_COLLECT;
            break;

        case STATE_DROWSY_COLLECT:
            // Each press during Phase 2 marks a nod peak
            if (!hatDataReady) {
                Serial.println("[WARN] No hat data yet — is hat powered on?");
                return;
            }
            if (nodCount < MAX_NODS) {
                float diffPitch = getDifferentialPitch();
                float dropRate = computeDropRate();

                nodPeaks[nodCount] = diffPitch;
                nodRates[nodCount] = dropRate;
                nodCount++;

                Serial.printf("[CAL] Nod %d/%d — peak=%.2f deg, rate=%.2f deg/s\n",
                              nodCount, NOD_COUNT_TARGET, diffPitch, dropRate);
                buzzerBeeps(1, 80, 0); // short single beep per nod

                if (nodCount >= NOD_COUNT_TARGET) {
                    // Enough nods collected — compute and save
                    Serial.println("[CAL] Nod collection complete. Saving...");
                    saveToNVS();
                    buzzerBeeps(3);
                    state = STATE_COMPLETE;
                    Serial.println("[CAL] Calibration complete. Flash DD firmware.");
                }
            }
            break;

        default:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────
void runRezero() {
    if (!hatDataReady) return; // wait for hat packets to arrive
    hatDataReady = false;

    // Accumulate raw differential (no offset applied yet —
    // we are computing the offset here)
    float rawDiff = hatPitch - carPitch;
    rezeroSumPitch += rawDiff;
    rezeroSumRoll += (hatRoll - carRoll);
    rezeroCount++;

    // End of re-zero window
    if (millis() - rezeroStartMs >= REZERO_DURATION_MS) {
        rezeroOffsetPitch = (float) (rezeroSumPitch / rezeroCount);
        rezeroOffsetRoll = (float) (rezeroSumRoll / rezeroCount);

        Serial.printf("[CAL] Re-zero complete. Offset pitch=%.3f roll=%.3f\n",
                      rezeroOffsetPitch, rezeroOffsetRoll);

        // Immediately begin Phase 1
        Serial.println("[CAL] Phase 1: drive normally for 3 minutes.");
        wMeanPitch = wM2Pitch = 0.0;
        wMeanRoll = wM2Roll = 0.0;
        wCount = 0;
        alertStartMs = millis();
        buzzerBeeps(1);
        state = STATE_ALERT_COLLECT;
    }
}

// ─────────────────────────────────────────────────────────────────
void runAlertCollect() {
    if (!hatDataReady) return;
    hatDataReady = false;

    float diffPitch = getDifferentialPitch();
    float diffRoll = getDifferentialRoll();

    // Update pitch history for rate computation
    updatePitchHistory(diffPitch);

    // Outlier rejection — discard samples from sharp turns,
    // sudden braking, or hat adjustment
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
    unsigned long elapsed = millis() - alertStartMs;
    static unsigned long lastLogMs = 0;
    if (elapsed - lastLogMs >= 30000) {
        lastLogMs = elapsed;
        Serial.printf("[CAL] Phase 1: %lu / %d seconds (%d samples)\n",
                      elapsed / 1000, ALERT_DURATION_MS / 1000, wCount);
    }

    // Phase 1 complete
    if (elapsed >= ALERT_DURATION_MS) {
        float stdPitch = (wCount > 1) ? sqrtf((float) (wM2Pitch / (wCount - 1))) : 0.0f;
        float stdRoll = (wCount > 1) ? sqrtf((float) (wM2Roll / (wCount - 1))) : 0.0f;

        Serial.println("[CAL] Phase 1 complete.");
        Serial.printf("  mean_alert_pitch = %.3f deg\n", (float) wMeanPitch);
        Serial.printf("  std_alert_pitch  = %.3f deg\n", stdPitch);
        Serial.printf("  mean_alert_roll  = %.3f deg\n", (float) wMeanRoll);
        Serial.printf("  std_alert_roll   = %.3f deg\n", stdRoll);
        Serial.println("  Press button to begin Phase 2 (drowsy nods).");

        // Save Phase 1 results to NVS immediately —
        // so they're not lost if Phase 2 is aborted
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
    if (!hatDataReady) return;
    hatDataReady = false;

    float diffPitch = getDifferentialPitch();

    // Keep history updated so drop rate is always
    // current when button is pressed
    updatePitchHistory(diffPitch);

    // Serial stream during Phase 2 so passenger can
    // see the signal and judge nod peaks visually
    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs > 200) {
        lastPrintMs = millis();
        Serial.printf("[LIVE] diff_pitch=%.2f  nods=%d/%d\n",
                      diffPitch, nodCount, NOD_COUNT_TARGET);
    }
}

// ─────────────────────────────────────────────────────────────────
void saveToNVS() {
    // Compute mean nod peak and mean drop rate from collected nods.
    // Simple mean — could use median for robustness but mean is
    // sufficient for NOD_COUNT_TARGET = 8 clean samples.
    float sumPeaks = 0.0f, sumRates = 0.0f;
    for (int i = 0; i < nodCount; i++) {
        sumPeaks += nodPeaks[i];
        sumRates += fabsf(nodRates[i]); // absolute value — rate is
        // negative (forward drop)
    }
    float meanDrowsyPitch = sumPeaks / nodCount;
    float meanDrowsyRate = sumRates / nodCount;

    Serial.println("[CAL] Drowsy calibration summary:");
    Serial.printf("  mean_drowsy_pitch = %.3f deg\n", meanDrowsyPitch);
    Serial.printf("  mean_drowsy_rate  = %.3f deg/s\n", meanDrowsyRate);

    // Print all individual nod observations for post-hoc analysis
    Serial.println("  Individual nods:");
    for (int i = 0; i < nodCount; i++) {
        Serial.printf("    [%d] peak=%.2f deg  rate=%.2f deg/s\n",
                      i, nodPeaks[i], fabsf(nodRates[i]));
    }

    // Save Phase 2 results — Phase 1 results already saved
    prefs.begin("dd_profile", false);
    prefs.putFloat("mean_drowsy_pitch", meanDrowsyPitch);
    prefs.putFloat("mean_drowsy_rate", meanDrowsyRate);
    prefs.putInt("nod_count", nodCount);
    prefs.putBool("phase2_done", true);
    prefs.end();

    Serial.println("[CAL] All values saved to NVS under namespace 'dd_profile'.");
    Serial.println("      Keys: mean_alert_pitch, std_alert_pitch,");
    Serial.println("             mean_alert_roll,  std_alert_roll,");
    Serial.println("             mean_drowsy_pitch, mean_drowsy_rate");
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
