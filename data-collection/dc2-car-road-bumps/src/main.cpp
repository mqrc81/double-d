// ================================================================
// DC2 — Bump Detection on Roads
// Car unit — ESP32-DevKitM-1 + MPU-6050
//
// PURPOSE
//   Collect angular rate statistics from real road bump events and
//   smooth road sections to empirically derive:
//     - BUMP_RATE_THRESH      (20th percentile of bump peak rate)
//     - BUMP_COOLDOWN_MS      (20th percentile of bump duration)
//     - isBump window (hat)   (P80 bump duration + 50 ms margin)
//     - stabilisation gate    (P80 between-bump baseline rate)
//
// HARDWARE  (same as car unit)
//   MPU-6050 on I2C (SDA=21, SCL=22)
//   Button on GPIO 26 (active LOW, internal pull-up) — matches car unit
//   LED    on GPIO 13
//   INTERRUPT_PIN GPIO 4                             — matches car unit
//
// BUTTON ROLES
//   Single press  →  "bump incoming" — arms a 3 s capture window.
//                    Firmware auto-detects the peak rate and duration
//                    within that window and logs them.
//   Double press  →  "clean road section" — logs the next 5 s of
//   (two presses     continuous rate data as baseline vibration.
//    within 400 ms)
//
// PROCEDURE
//   Passenger operates the button; driver focuses on the road.
//   1. Power on. Wait 30 s for DMP warmup (LED blinks slowly).
//   2. Drive to the test route. No calibration step needed.
//   3. For each planned bump (speed bump, pothole, cobbles):
//      a. Passenger presses button ONCE ~1–2 s before the bump.
//      b. Drive over the bump normally.
//      c. LED triple-flashes to confirm the event was captured.
//      d. Target: TARGET_BUMPS bump events total (default 30).
//   4. On smooth tarmac sections between bumps:
//      a. Passenger double-presses button to start a 5 s baseline window.
//      b. LED fast-blinks during the 5 s window.
//      c. Target: TARGET_BASELINES baseline windows total (default 15).
//   5. After both targets are reached, results print automatically.
//      If you finish bumps before baselines (or vice versa), the
//      firmware waits for the remaining target then prints.
//   6. Copy the Serial Monitor output.
//
// TARGETING
//   30 bump events  → stable P20 (6 samples in lower tail)
//   15 baseline windows × 100 samples each = 1500 baseline samples
//   → stable P80 for vibration characterisation
// ================================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// ── PINS ─────────────────────────────────────────────────────────
#define INTERRUPT_PIN   4    // matches car unit
#define BUTTON_PIN      26   // matches car unit
#define LED_PIN         13

// ── SAMPLE RATE ──────────────────────────────────────────────────
// mpu.setRate(49) → DMP output = 1000 / (1+49) = 20 Hz
#define SAMPLE_RATE_HZ  20

// ── DATA COLLECTION CONFIG ────────────────────────────────────────
#define TARGET_BUMPS        30    // bump events to collect
#define TARGET_BASELINES    15    // smooth-road windows to collect
#define BUMP_WINDOW_MS    3000    // auto-capture window after single press
#define BASELINE_WINDOW_MS 5000  // baseline recording window after double press
#define BASELINE_SAMPLES_MAX 1500 // 15 windows × 100 samples — heap buffer

// Double-press detection: two presses within this window = double press
#define DOUBLE_PRESS_MS   400

// Minimum rate to qualify as a bump peak (guards against capturing
// a non-event if button was pressed early on smooth road).
// Set conservatively low — the P20 derivation will determine the
// real threshold; this only filters out flat-road noise.
#define BUMP_MIN_RATE    10.0f   // °/s

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

// ── BUMP EVENT STORAGE ────────────────────────────────────────────
float bumpPeakRate[TARGET_BUMPS]; // peak °/s during bump window
float bumpDuration[TARGET_BUMPS]; // ms from rate crossing BUMP_MIN_RATE
// to return within 10% of pre-bump rate
int bumpCount = 0;

// ── BASELINE STORAGE ─────────────────────────────────────────────
// Flat array accumulating all samples from all baseline windows.
float baselineRates[BASELINE_SAMPLES_MAX];
int baselineSampleCount = 0;
int baselineWindowCount = 0;

// ── BUMP CAPTURE STATE ────────────────────────────────────────────
bool bumpArmed = false;
unsigned long bumpWindowStart = 0;
float bumpPreRate = 0.0f; // median rate in 500 ms before press
float bumpPeak = 0.0f;
unsigned long bumpOnsetTime = 0;
bool bumpOnsetFound = false;
unsigned long bumpEndTime = 0;
bool bumpEndFound = false;

// rolling pre-bump rate estimate: keep last 10 samples (500 ms at 20 Hz)
#define PRE_BUMP_WINDOW 10
float preBumpBuf[PRE_BUMP_WINDOW];
int preBumpIdx = 0;
bool preBumpFull = false;

// ── BASELINE CAPTURE STATE ────────────────────────────────────────
bool baselineArmed = false;
unsigned long baselineStart = 0;

// ── BUTTON DEBOUNCE & DOUBLE-PRESS ───────────────────────────────
bool lastBtn = HIGH;
unsigned long lastDebounce = 0;
unsigned long firstPressTime = 0;
bool waitingSecond = false;
#define DEBOUNCE_MS  50

// ── HELPERS ───────────────────────────────────────────────────────
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

float percentile(float *sorted, int n, float p) {
    if (n == 1) return sorted[0];
    float idx = (p / 100.0f) * (n - 1);
    int lo = (int) idx;
    int hi = lo + 1;
    if (hi >= n) return sorted[n - 1];
    float frac = idx - lo;
    return sorted[lo] + frac * (sorted[hi] - sorted[lo]);
}

// tmp buffer sized to the largest dataset (baseline samples)
float _tmp[BASELINE_SAMPLES_MAX];

void printPercentileTable(const char *label, float *data, int n) {
    for (int i = 0; i < n; i++) _tmp[i] = data[i];
    sortAsc(_tmp, n);

    Serial.print("\n--- ");
    Serial.print(label);
    Serial.println(" ---");
    Serial.println("P\tValue");
    for (int p = 5; p <= 95; p += 5) {
        Serial.print("P");
        Serial.print(p);
        Serial.print("\t");
        Serial.println(percentile(_tmp, n, p), 2);
    }
}

float preBumpMedian() {
    int n = preBumpFull ? PRE_BUMP_WINDOW : preBumpIdx;
    if (n == 0) return 0.0f;
    float tmp[PRE_BUMP_WINDOW];
    for (int i = 0; i < n; i++) tmp[i] = preBumpBuf[i];
    sortAsc(tmp, n);
    return percentile(tmp, n, 50.0f);
}

void printResults() {
    Serial.println("\n\n========================================");
    Serial.print(" DC2 RESULTS — ");
    Serial.print(bumpCount);
    Serial.print(" bumps, ");
    Serial.print(baselineWindowCount);
    Serial.println(" baseline windows");
    Serial.println("========================================");

    printPercentileTable(
        "Bump peak rate (deg/s)  → BUMP_RATE_THRESH = P20",
        bumpPeakRate, bumpCount);
    printPercentileTable(
        "Bump duration (ms)  → BUMP_COOLDOWN_MS = P20 / isBump window = P80+50ms",
        bumpDuration, bumpCount);
    printPercentileTable(
        "Baseline road vibration rate (deg/s)  → stabilisation gate = P80",
        baselineRates, baselineSampleCount);

    // sort copies for key derived values
    float tmpR[TARGET_BUMPS], tmpD[TARGET_BUMPS];
    for (int i = 0; i < bumpCount; i++) {
        tmpR[i] = bumpPeakRate[i];
        tmpD[i] = bumpDuration[i];
    }
    sortAsc(tmpR, bumpCount);
    sortAsc(tmpD, bumpCount);

    float tmpB[BASELINE_SAMPLES_MAX];
    for (int i = 0; i < baselineSampleCount; i++) tmpB[i] = baselineRates[i];
    sortAsc(tmpB, baselineSampleCount);

    float p20rate = percentile(tmpR, bumpCount, 20.0f);
    float p20dur = percentile(tmpD, bumpCount, 20.0f);
    float p80dur = percentile(tmpD, bumpCount, 80.0f);
    float p80vib = percentile(tmpB, baselineSampleCount, 80.0f);

    Serial.println("\n--- KEY DERIVED VALUES ---");
    Serial.print("BUMP_RATE_THRESH     (P20 bump peak rate)     = ");
    Serial.print(p20rate, 2);
    Serial.println(" deg/s");
    Serial.print("BUMP_COOLDOWN_MS     (P20 bump duration)      = ");
    Serial.print(p20dur, 0);
    Serial.println(" ms");
    Serial.print("isBump window (hat)  (P80 dur + 50 ms margin) = ");
    Serial.print(p80dur + 50.0f, 0);
    Serial.println(" ms");
    Serial.print("Stabilisation gate   (P80 baseline vib rate)  = ");
    Serial.print(p80vib, 2);
    Serial.println(" deg/s");

    Serial.println("\nRaw bump events (peak_rate_dps, duration_ms):");
    for (int i = 0; i < bumpCount; i++) {
        Serial.print(i + 1);
        Serial.print("\t");
        Serial.print(bumpPeakRate[i], 2);
        Serial.print("\t");
        Serial.println((int) bumpDuration[i]);
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
    Serial.println("DMP warmup — 30 s. Mount unit in car and start driving.");
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
    Serial.println("Warmup done. Ready to collect.");
    Serial.print("Targets: ");
    Serial.print(TARGET_BUMPS);
    Serial.print(" bump events, ");
    Serial.print(TARGET_BASELINES);
    Serial.println(" baseline windows.");
    Serial.println("Single press = arm bump capture (~1-2 s before bump).");
    Serial.println("Double press = start 5 s baseline window (smooth road only).");
}

// ── LOOP ──────────────────────────────────────────────────────────
void loop() {
    if (!dmpReady) return;

    unsigned long now = millis();

    // ── button edge detection + double-press logic ────────────────
    bool btnNow = digitalRead(BUTTON_PIN);
    bool singlePress = false;
    bool doublePress = false;

    if (btnNow == LOW && lastBtn == HIGH && (now - lastDebounce > DEBOUNCE_MS)) {
        lastDebounce = now;

        if (waitingSecond && (now - firstPressTime <= DOUBLE_PRESS_MS)) {
            doublePress = true;
            waitingSecond = false;
        } else {
            // first press of a potential double — wait to see if second arrives
            firstPressTime = now;
            waitingSecond = true;
        }
    }
    lastBtn = btnNow;

    // if waiting for second press and window has expired → it was single
    if (waitingSecond && (now - firstPressTime > DOUBLE_PRESS_MS)) {
        singlePress = true;
        waitingSecond = false;
    }

    // ── sensor read ───────────────────────────────────────────────
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // car unit: pitch = road angle (sensor mounted flat in car)
    float pitch = -(ypr[2] * 180.0f / M_PI);
    float roll = ypr[1] * 180.0f / M_PI;

    static float lastPitch = 0.0f;
    static float lastRoll = 0.0f;
    static unsigned long lastSampleTime = 0;
    static bool firstSample = true;

    if (firstSample) {
        firstSample = false;
        lastPitch = pitch;
        lastRoll = roll;
        lastSampleTime = now;
        return;
    }

    float dt = (now - lastSampleTime) / 1000.0f;
    if (dt > 1.0f) dt = 1.0f / SAMPLE_RATE_HZ;
    float pitchRate = (dt > 0) ? fabsf(pitch - lastPitch) / dt : 0.0f;
    float rollRate = (dt > 0) ? fabsf(roll - lastRoll) / dt : 0.0f;
    float totalRate = max(pitchRate, rollRate); // bump can be in either axis
    lastPitch = pitch;
    lastRoll = roll;
    lastSampleTime = now;

    // ── rolling pre-bump rate buffer ──────────────────────────────
    preBumpBuf[preBumpIdx] = totalRate;
    preBumpIdx = (preBumpIdx + 1) % PRE_BUMP_WINDOW;
    if (preBumpIdx == 0) preBumpFull = true;

    // ── single press: arm bump capture ───────────────────────────
    if (singlePress && !bumpArmed && bumpCount < TARGET_BUMPS) {
        bumpArmed = true;
        bumpWindowStart = now;
        bumpPreRate = preBumpMedian();
        bumpPeak = 0.0f;
        bumpOnsetFound = false;
        bumpEndFound = false;
        bumpOnsetTime = 0;
        bumpEndTime = 0;
        Serial.print("Bump armed [");
        Serial.print(bumpCount + 1);
        Serial.print("/");
        Serial.print(TARGET_BUMPS);
        Serial.print("]  pre-rate=");
        Serial.print(bumpPreRate, 1);
        Serial.println("°/s");
    }

    // ── double press: start baseline window ───────────────────────
    if (doublePress && !baselineArmed && baselineWindowCount < TARGET_BASELINES) {
        baselineArmed = true;
        baselineStart = now;
        Serial.print("Baseline window ");
        Serial.print(baselineWindowCount + 1);
        Serial.print("/");
        Serial.print(TARGET_BASELINES);
        Serial.println(" started — 5 s smooth road...");
    }

    // ── bump capture window ───────────────────────────────────────
    if (bumpArmed) {
        digitalWrite(LED_PIN, (now / 100) % 2); // rapid blink = armed

        // track onset: first sample exceeding BUMP_MIN_RATE
        if (!bumpOnsetFound && totalRate > BUMP_MIN_RATE) {
            bumpOnsetFound = true;
            bumpOnsetTime = now;
        }

        // track peak
        if (totalRate > bumpPeak) {
            bumpPeak = totalRate;
        }

        // track end: rate returns within 10% above pre-bump level
        // (or within absolute 3°/s if pre-bump rate was very low)
        float returnThresh = bumpPreRate * 1.1f + 3.0f;
        if (bumpOnsetFound && !bumpEndFound && totalRate < returnThresh) {
            bumpEndFound = true;
            bumpEndTime = now;
        }

        // window expired or end found — log the event
        bool windowExpired = (now - bumpWindowStart) >= BUMP_WINDOW_MS;
        if (windowExpired || (bumpEndFound && bumpOnsetFound)) {
            if (bumpPeak >= BUMP_MIN_RATE) {
                unsigned long dur = bumpEndFound
                                        ? (bumpEndTime - bumpOnsetTime)
                                        : BUMP_WINDOW_MS; // cap if bump never settled

                bumpPeakRate[bumpCount] = bumpPeak;
                bumpDuration[bumpCount] = (float) dur;
                bumpCount++;

                // triple flash confirmation
                for (int i = 0; i < 3; i++) {
                    digitalWrite(LED_PIN, HIGH);
                    delay(80);
                    digitalWrite(LED_PIN, LOW);
                    delay(80);
                }

                Serial.print("Bump ");
                Serial.print(bumpCount);
                Serial.print("/");
                Serial.print(TARGET_BUMPS);
                Serial.print("  peak=");
                Serial.print(bumpPeak, 1);
                Serial.print("°/s  dur=");
                Serial.print((int) dur);
                Serial.println("ms");
            } else {
                Serial.println("Bump window expired — no event detected. Try earlier press.");
            }
            bumpArmed = false;
            digitalWrite(LED_PIN, LOW);
        }
    }

    // ── baseline window ───────────────────────────────────────────
    if (baselineArmed) {
        digitalWrite(LED_PIN, (now / 150) % 2); // fast blink = baseline

        if (baselineSampleCount < BASELINE_SAMPLES_MAX) {
            baselineRates[baselineSampleCount++] = totalRate;
        }

        if ((now - baselineStart) >= BASELINE_WINDOW_MS) {
            baselineArmed = false;
            baselineWindowCount++;
            digitalWrite(LED_PIN, LOW);
            Serial.print("Baseline window ");
            Serial.print(baselineWindowCount);
            Serial.print(" complete  (total samples: ");
            Serial.print(baselineSampleCount);
            Serial.println(")");
        }
    }

    // ── check if both targets are reached ────────────────────────
    if (bumpCount >= TARGET_BUMPS && baselineWindowCount >= TARGET_BASELINES) {
        printResults();
        digitalWrite(LED_PIN, HIGH);
        while (true) delay(1000);
    }

    // ── live monitor (every 10 samples) ───────────────────────────
    static int dbg = 0;
    if (++dbg >= 10) {
        dbg = 0;
        Serial.print("rate=");
        Serial.print(totalRate, 1);
        Serial.print("°/s  bumps=");
        Serial.print(bumpCount);
        Serial.print("/");
        Serial.print(TARGET_BUMPS);
        Serial.print("  base=");
        Serial.print(baselineWindowCount);
        Serial.print("/");
        Serial.println(TARGET_BASELINES);
    }
}