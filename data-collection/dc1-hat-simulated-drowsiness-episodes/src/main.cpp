// ================================================================
// DC1 — Simulated Drowsiness Episodes
// Hat unit — ESP32-DevKitM-1 + MPU-6050
//
// PURPOSE
//   Collect pitch statistics from deliberate slow head-drop events
//   to empirically derive:
//     - PITCH_THRESHOLD   (10th percentile of peak relative pitch)
//     - DROP_RATE_THRESH  (90th percentile of angular rate during drops)
//     - ALARM_DURATION_MS floor (5th percentile of drop duration)
//
// HARDWARE  (same as hat unit)
//   MPU-6050 on I2C (SDA=21, SCL=22)
//   Button on GPIO 0 (active LOW, internal pull-up)
//   LED   on GPIO 13
//   INTERRUPT_PIN GPIO 2
//
// PROCEDURE
//   1. Power on. Wait 30 s for DMP warmup (LED blinks slowly).
//   2. Sit upright in natural driving posture.
//   3. Hold button for baseline calibration (5 s, LED fast-blinks).
//   4. Perform a slow deliberate head drop (mimic drowsy slump).
//      Press button ONCE at the moment your head reaches its
//      lowest point. The firmware captures the peak relative pitch,
//      the angular rate at that moment, and the duration since the
//      drop started (detected automatically by threshold crossing).
//   5. Return head upright. Repeat from step 4.
//   6. After TARGET_EVENTS drops, percentile table prints
//      automatically and the LED stays solid.
//   7. Copy the Serial Monitor output.
//
// TARGETING 40 events — enough for stable 5th/95th percentile
// estimates (rule of thumb: n >= 20 / p for extreme percentiles;
// 40 >= 20/0.05 = 400 is not feasible manually, so we use 40 as a
// practical minimum giving stable 10th/90th with ~4 samples in the
// tail, and treat the 5th percentile estimate as approximate).
// ================================================================

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ── PINS ─────────────────────────────────────────────────────────
#define INTERRUPT_PIN   2
#define BUTTON_PIN      0    // active LOW, internal pull-up
#define LED_PIN         13

// ── SAMPLE RATE ──────────────────────────────────────────────────
// mpu.setRate(49) → DMP output = 1000 / (1+49) = 20 Hz
#define SAMPLE_RATE_HZ  20

// ── DATA COLLECTION CONFIG ────────────────────────────────────────
#define TARGET_EVENTS       40    // stop after this many logged drops
#define CALIB_DURATION_MS   5000  // 5 s baseline calibration window
#define CALIB_RATE_GATE     5.0f  // °/s — reject samples if moving faster
#define CALIB_MIN_SAMPLES   30    // minimum still samples required

// A drop is "started" when relative pitch exceeds this soft trigger.
// Set low so we catch the very beginning of slow slumps.
// NOT the threshold being measured — just a drop-onset detector.
#define DROP_ONSET_DEG      3.0f  // °  above baseline → drop has started
#define DROP_RATE_MAX      80.0f  // °/s — ignore jerks (not drowsy drops)

// ── MPU ──────────────────────────────────────────────────────────
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// ── CALIBRATION ───────────────────────────────────────────────────
float baselinePitch = 0.0f;
bool isCalibrated = false;
bool calibInProgress = false;
unsigned long calibStart = 0;
float calibSum = 0.0f;
int calibCount = 0;

// ── DROP STATE MACHINE ────────────────────────────────────────────
bool dropActive = false;
unsigned long dropStart = 0;
float dropPeakPitch = 0.0f;
float dropPeakRate = 0.0f;

// ── COLLECTED DATA ────────────────────────────────────────────────
// Three arrays: peak relative pitch, angular rate at capture,
// and duration from onset to button press.
float evtPitch[TARGET_EVENTS];
float evtRate[TARGET_EVENTS];
float evtDur[TARGET_EVENTS]; // ms → stored as float for sorting
int evtCount = 0;

// ── BUTTON DEBOUNCE ───────────────────────────────────────────────
bool lastBtn = HIGH;
unsigned long lastDebounce = 0;
#define DEBOUNCE_MS  50

// ── HELPERS ───────────────────────────────────────────────────────
// In-place insertion sort (small N, readability > speed)
void sortAsc(float *arr, int n) {
    for (int i = 1; i < n; i++) {
        float key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// Linear-interpolation percentile (same method as numpy default)
float percentile(float *sorted, int n, float p) {
    if (n == 1) return sorted[0];
    float idx = (p / 100.0f) * (n - 1);
    int lo = (int) idx;
    int hi = lo + 1;
    if (hi >= n) return sorted[n - 1];
    float frac = idx - lo;
    return sorted[lo] + frac * (sorted[hi] - sorted[lo]);
}

void printPercentileTable(const char *label, float *data, int n) {
    // copy so we can sort without destroying originals
    float tmp[TARGET_EVENTS];
    for (int i = 0; i < n; i++) tmp[i] = data[i];
    sortAsc(tmp, n);

    Serial.print("\n--- ");
    Serial.print(label);
    Serial.println(" ---");
    Serial.println("P\tValue");
    for (int p = 5; p <= 95; p += 5) {
        Serial.print("P");
        Serial.print(p);
        Serial.print("\t");
        Serial.println(percentile(tmp, n, p), 2);
    }
}

void printResults() {
    Serial.println("\n\n========================================");
    Serial.println(" DC1 RESULTS — " + String(evtCount) + " events");
    Serial.println("========================================");

    printPercentileTable("Peak Relative Pitch (deg)  → PITCH_THRESHOLD = P10",
                         evtPitch, evtCount);
    printPercentileTable("Angular Rate at capture (deg/s)  → DROP_RATE_THRESH = P90",
                         evtRate, evtCount);
    printPercentileTable("Drop Duration ms  → ALARM_DURATION_MS floor = P5",
                         evtDur, evtCount);

    Serial.println("\n--- KEY DERIVED VALUES ---");

    float tmpP[TARGET_EVENTS], tmpR[TARGET_EVENTS], tmpD[TARGET_EVENTS];
    for (int i = 0; i < evtCount; i++) {
        tmpP[i] = evtPitch[i];
        tmpR[i] = evtRate[i];
        tmpD[i] = evtDur[i];
    }
    sortAsc(tmpP, evtCount);
    sortAsc(tmpR, evtCount);
    sortAsc(tmpD, evtCount);

    Serial.print("PITCH_THRESHOLD  (P10 peak pitch)    = ");
    Serial.println(percentile(tmpP, evtCount, 10.0f), 2);
    Serial.print("DROP_RATE_THRESH (P90 angular rate)  = ");
    Serial.println(percentile(tmpR, evtCount, 90.0f), 2);
    Serial.print("ALARM_DURATION floor (P5 duration)   = ");
    Serial.print(percentile(tmpD, evtCount, 5.0f), 0);
    Serial.println(" ms");

    Serial.println("\nRaw events (pitch_deg, rate_dps, dur_ms):");
    for (int i = 0; i < evtCount; i++) {
        Serial.print(i + 1);
        Serial.print("\t");
        Serial.print(evtPitch[i], 2);
        Serial.print("\t");
        Serial.print(evtRate[i], 2);
        Serial.print("\t");
        Serial.println((int) evtDur[i]);
    }
    Serial.println("========================================");
}

// ── SETUP ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    Wire.begin(21, 22);
    Wire.setClock(400000);

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection FAILED — check wiring");
        while (true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }
    Serial.println("MPU6050 OK");

    devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        Serial.print("DMP init failed, code: ");
        Serial.println(devStatus);
        while (true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setRate(49); // 20 Hz
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    // ── 30 s DMP warmup ──────────────────────────────────────────
    Serial.println("DMP warmup — 30 s. Sit still.");
    unsigned long warmupStart = millis();
    while (millis() - warmupStart < 30000) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
        digitalWrite(LED_PIN, (millis() / 500) % 2);
    }
    digitalWrite(LED_PIN, LOW);
    Serial.println("Warmup done.");
    Serial.println("Press button to start baseline calibration.");
    Serial.println("Sit upright in natural driving posture and hold still for 5 s.");
}

// ── LOOP ──────────────────────────────────────────────────────────
void loop() {
    if (!dmpReady) return;

    unsigned long now = millis();

    // ── button edge detection ─────────────────────────────────────
    bool btnNow = digitalRead(BUTTON_PIN);
    bool btnPressed = false;
    if (btnNow == LOW && lastBtn == HIGH && (now - lastDebounce > DEBOUNCE_MS)) {
        lastDebounce = now;
        btnPressed = true;
    }
    lastBtn = btnNow;

    // ── sensor read ───────────────────────────────────────────────
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = -(ypr[2] * 180.0f / M_PI); // forward tilt positive

    static float lastPitch = 0.0f;
    static unsigned long lastSampleTime = 0;
    static bool firstSample = true;

    if (firstSample) {
        firstSample = false;
        lastPitch = pitch;
        lastSampleTime = now;
        return;
    }

    float dt = (now - lastSampleTime) / 1000.0f;
    if (dt > 1.0f) dt = 1.0f / SAMPLE_RATE_HZ;
    float pitchRate = (dt > 0) ? fabsf(pitch - lastPitch) / dt : 0.0f;
    lastPitch = pitch;
    lastSampleTime = now;

    // ── STATE: WAITING FOR CALIBRATION ────────────────────────────
    if (!isCalibrated && !calibInProgress) {
        digitalWrite(LED_PIN, (now / 500) % 2); // slow blink = waiting
        if (btnPressed) {
            calibInProgress = true;
            calibStart = now;
            calibSum = 0.0f;
            calibCount = 0;
            Serial.println("Calibrating — hold still...");
        }
        return;
    }

    // ── STATE: CALIBRATING ────────────────────────────────────────
    if (calibInProgress) {
        digitalWrite(LED_PIN, (now / 150) % 2); // fast blink = calibrating
        if (now - calibStart < CALIB_DURATION_MS) {
            if (pitchRate < CALIB_RATE_GATE) {
                calibSum += pitch;
                calibCount++;
            }
        } else {
            // calibration window done
            if (calibCount >= CALIB_MIN_SAMPLES) {
                baselinePitch = calibSum / calibCount;
                isCalibrated = true;
                calibInProgress = false;
                digitalWrite(LED_PIN, LOW);
                Serial.print("Baseline pitch = ");
                Serial.print(baselinePitch, 2);
                Serial.print(" deg  (");
                Serial.print(calibCount);
                Serial.println(" samples)");
                Serial.println();
                Serial.println("Ready. Perform a slow head drop, then press button at lowest point.");
                Serial.println("Events remaining: " + String(TARGET_EVENTS - evtCount));
            } else {
                calibInProgress = false;
                Serial.println("Calibration failed — too few still samples. Press button to retry.");
            }
        }
        return;
    }

    // ── STATE: COLLECTING DROPS ───────────────────────────────────
    float relPitch = pitch - baselinePitch;

    // drop onset: head has moved past onset threshold (slow movement only)
    if (!dropActive && relPitch > DROP_ONSET_DEG && pitchRate < DROP_RATE_MAX) {
        dropActive = true;
        dropStart = now;
        dropPeakPitch = relPitch;
        dropPeakRate = pitchRate;
    }

    // while drop is active: track peak pitch and rate
    if (dropActive) {
        if (relPitch > dropPeakPitch) {
            dropPeakPitch = relPitch;
            dropPeakRate = pitchRate; // rate at new peak
        }

        // button press = capture event at lowest point
        if (btnPressed) {
            unsigned long dur = now - dropStart;

            evtPitch[evtCount] = dropPeakPitch;
            evtRate[evtCount] = dropPeakRate;
            evtDur[evtCount] = (float) dur;
            evtCount++;

            // brief LED confirmation flash
            for (int i = 0; i < 3; i++) {
                digitalWrite(LED_PIN, HIGH);
                delay(80);
                digitalWrite(LED_PIN, LOW);
                delay(80);
            }

            Serial.print("Event ");
            Serial.print(evtCount);
            Serial.print("/");
            Serial.print(TARGET_EVENTS);
            Serial.print("  peak=");
            Serial.print(dropPeakPitch, 1);
            Serial.print("°  rate=");
            Serial.print(dropPeakRate, 1);
            Serial.print("°/s  dur=");
            Serial.print(dur);
            Serial.println("ms");

            dropActive = false;
            dropPeakPitch = 0.0f;
            dropPeakRate = 0.0f;

            if (evtCount >= TARGET_EVENTS) {
                printResults();
                // halt — hold LED solid
                digitalWrite(LED_PIN, HIGH);
                while (true) delay(1000);
            } else {
                Serial.println("Return head upright, then do next drop.");
            }
        }

        // if head returns to upright before button pressed → abort this rep
        // (operator started but didn't complete the drop)
        if (relPitch < 0.5f) {
            dropActive = false;
            dropPeakPitch = 0.0f;
            dropPeakRate = 0.0f;
            Serial.println("Drop aborted (head returned before button press).");
        }
    }

    // ── live monitor (every 10 samples) ───────────────────────────
    static int dbg = 0;
    if (++dbg >= 10) {
        dbg = 0;
        Serial.print("rel=");
        Serial.print(relPitch, 1);
        Serial.print("°  rate=");
        Serial.print(pitchRate, 1);
        Serial.print("°/s  drop=");
        Serial.println(dropActive ? "ACTIVE" : "idle");
    }
}
