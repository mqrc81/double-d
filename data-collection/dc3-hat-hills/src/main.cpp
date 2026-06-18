// ================================================================
// DC3 — Hill Driving  (HAT UNIT)
// Hat unit — ESP32-DevKitM-1 + MPU-6050
//
// PURPOSE
//   Calibrate baseline pitch on flat ground. Receive two messages
//   from car unit when it has settled on a gradient:
//     EVENT_ROAD_TILT  — settled car pitch (°)
//     EVENT_STABLE_MS  — ms from gradient entry to stable
//   Read own relative pitch at that instant. Log all values and halt.
//
//   Logs (one entry per run, aggregated offline across all hills):
//     car_pitch        — settled road angle (°)
//     hat_relPitch     — head pitch relative to flat-ground baseline (°)
//     hillOffset ratio — hat_relPitch / car_pitch
//     stableElapsed    — candidate ROAD_STABLE_MS value (ms)
//
//   Final values derived offline:
//     hillOffset factor = mean(hillOffset ratio) across all runs
//     ROAD_STABLE_MS    = P80(stableElapsed) across all runs
//
// HARDWARE  (identical to hat production unit)
//   MPU-6050  on I2C (SDA=21, SCL=22) — explicit Wire.begin(21,22)
//   Button    GPIO 0   (active LOW, internal pull-up)
//   LED       GPIO 13
//   INTERRUPT GPIO 2
//
// ESP-NOW
//   Hat receives only. No sending. No peer registration needed.
//   Communication direction unchanged from production (car → hat).
//
// PROCEDURE
//   Both units powered on simultaneously.
//
//   1. Power on hat unit. Note MAC printed to Serial.
//      Update hatMAC[] in car DC3 firmware if not already done.
//   2. Wait 30 s DMP warmup (LED slow-blinks).
//      Sit in car on FLAT ground in natural driving posture.
//   3. Press button ONCE to start 5 s baseline calibration.
//      LED fast-blinks during calibration. Do this only once
//      per power cycle — baseline is reused across all hills.
//   4. Calibration succeeds → LED goes SOLID ON.
//      This is the signal to the car operator to proceed.
//   5. Car operator arms hill capture and drives onto gradient.
//      Hat waits silently, logging live pitch to Serial.
//   6. Car sends EVENT_ROAD_TILT then EVENT_STABLE_MS.
//      Hat logs result, LED triple-flashes, then returns to
//      SOLID ON — ready for next hill, no reflash needed.
//   7. Car operator reflashes car unit, repeats from step 5
//      for each subsequent hill.
//   8. Copy hat Serial output after all hills are done.
//
// CALIBRATE ONCE PER POWER CYCLE. Hat does not need reflashing.
// ================================================================

#include "I2Cdev.h"
#include <esp_now.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "WiFi.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// ── PINS  (matches hat production firmware exactly) ───────────────
#define INTERRUPT_PIN   2
#define BUTTON_PIN      0
#define LED_PIN         13

// ── SAMPLE RATE ───────────────────────────────────────────────────
#define SAMPLE_RATE_HZ  20   // mpu.setRate(49) → 1000/(1+49) = 20 Hz

// ── CALIBRATION CONFIG  (matches hat production firmware) ─────────
#define CALIB_DURATION_MS      5000
#define CALIB_SAMPLES_REQUIRED 50
#define CALIB_RATE_GATE        5.0f   // °/s — reject moving samples

// ── MESSAGE STRUCT  (identical to production) ─────────────────────
typedef struct {
    uint8_t eventType;
    float magnitude;
} ShoulderMessage;

#define EVENT_BUTTON     1
#define EVENT_BUMP       2
#define EVENT_ROAD_TILT  3
#define EVENT_STABLE_MS  4   // DC3 only: carries stableElapsed from car

// ── RECEIVED DATA FROM CAR ────────────────────────────────────────
// Both flags set from ESP-NOW callback; consumed in loop.
volatile bool gotRoadTilt = false;
volatile float carPitch = 0.0f;
volatile bool gotStableMs = false;
volatile float stableElapsed = 0.0f;

// ── MPU ───────────────────────────────────────────────────────────
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

// ── CALIBRATION STATE ─────────────────────────────────────────────
float baselinePitch = 0.0f;
bool isCalibrated = false;
bool calibInProgress = false;
unsigned long calibStart = 0;
float calibSum = 0.0f;
int calibCount = 0;

// ── RUN STATE ─────────────────────────────────────────────────────
// Per-run state — reset after each hill so hat can capture
// multiple hills without reflashing.
int runCount = 0;
float capturedCarPitch = 0.0f;
float hatPitchAtTilt = 0.0f;
bool tiltSnapTaken = false;

// ── BUTTON DEBOUNCE ───────────────────────────────────────────────
bool lastBtn = HIGH;
unsigned long lastDebounce = 0;
#define DEBOUNCE_MS  50

// ── ESP-NOW RECEIVE CALLBACK ──────────────────────────────────────
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(ShoulderMessage)) return;
    ShoulderMessage msg;
    memcpy(&msg, data, sizeof(ShoulderMessage));

    if (msg.eventType == EVENT_ROAD_TILT) {
        carPitch = msg.magnitude;
        gotRoadTilt = true;
    } else if (msg.eventType == EVENT_STABLE_MS) {
        stableElapsed = msg.magnitude;
        gotStableMs = true;
    }
    // EVENT_BUTTON and EVENT_BUMP ignored in DC3
}

// ── HELPERS ───────────────────────────────────────────────────────
void ledFlash(int n, int onMs, int offMs) {
    for (int i = 0; i < n; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(onMs);
        digitalWrite(LED_PIN, LOW);
        delay(offMs);
    }
}

void printResult(float cp, float hp, float se) {
    float ratio = (fabsf(cp) > 0.1f) ? hp / cp : 0.0f;

    Serial.println("\n========================================");
    Serial.print(" DC3 RESULT — run #");
    Serial.println(runCount);
    Serial.println("========================================");
    Serial.print("Car settled pitch      = ");
    Serial.print(cp, 2);
    Serial.println(" deg");
    Serial.print("Hat relative pitch     = ");
    Serial.print(hp, 2);
    Serial.println(" deg");
    Serial.print("hillOffset ratio       = ");
    Serial.print(ratio, 3);
    Serial.println("  (hat_relPitch / car_pitch)");
    Serial.print("Gradient entry→stable  = ");
    Serial.print((unsigned long) se);
    Serial.println(" ms  → candidate ROAD_STABLE_MS");
    Serial.println("----------------------------------------");
    Serial.println("Aggregate across all hill runs:");
    Serial.println("  hillOffset factor = mean(ratio)");
    Serial.println("  ROAD_STABLE_MS    = P80(stableElapsed)");
    Serial.println("========================================");
}

// ── SETUP ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    // explicit I2C pins — do not rely on defaults
    Wire.begin(21, 22);
    Wire.setClock(400000);

    WiFi.mode(WIFI_MODE_STA);

    Serial.print("Hat MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED");
        while (true) { ledFlash(1, 100, 100); }
    }
    // receive only — no send callback, no peer registration
    esp_now_register_recv_cb(onDataReceived);

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 FAILED");
        while (true) { ledFlash(1, 100, 100); }
    }
    Serial.println("MPU6050 OK");

    devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        Serial.print("DMP init failed: ");
        Serial.println(devStatus);
        while (true) { ledFlash(1, 100, 100); }
    }

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setRate(49);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    Serial.println("DMP warmup 30 s — sit in car on flat ground.");
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
    Serial.println("Warmup done. Press button to calibrate baseline.");
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

    float pitch = -(ypr[2] * 180.0f / M_PI);

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
        digitalWrite(LED_PIN, (now / 500) % 2);
        if (btnPressed) {
            calibInProgress = true;
            calibStart = now;
            calibSum = 0.0f;
            calibCount = 0;
            Serial.println("Calibrating — hold perfectly still...");
        }
        return;
    }

    // ── STATE: CALIBRATING ────────────────────────────────────────
    if (calibInProgress) {
        digitalWrite(LED_PIN, (now / 150) % 2);

        if (now - calibStart < CALIB_DURATION_MS) {
            if (pitchRate < CALIB_RATE_GATE) {
                calibSum += pitch;
                calibCount++;
            }
        } else {
            if (calibCount >= CALIB_SAMPLES_REQUIRED) {
                baselinePitch = calibSum / calibCount;
                isCalibrated = true;
                calibInProgress = false;

                // solid LED = ready signal to car operator
                digitalWrite(LED_PIN, HIGH);

                Serial.print("Calibration OK. Baseline = ");
                Serial.print(baselinePitch, 2);
                Serial.print(" deg  (");
                Serial.print(calibCount);
                Serial.println(" samples)");
                Serial.println("LED solid — car operator may begin hill capture.");
            } else {
                calibInProgress = false;
                digitalWrite(LED_PIN, LOW);
                Serial.println("Calibration failed — too few still samples.");
                Serial.println("Press button to retry.");
            }
        }
        return;
    }

    // ── STATE: CALIBRATED — waiting for car messages ──────────────
    if (isCalibrated) {
        // atomically read and clear both volatile flags
        bool haveRoadTilt = false;
        bool haveStableMs = false;
        float cp = 0.0f;
        float se = 0.0f;

        noInterrupts();
        if (gotRoadTilt) {
            haveRoadTilt = true;
            cp = carPitch;
            gotRoadTilt = false;
        }
        if (gotStableMs) {
            haveStableMs = true;
            se = stableElapsed;
            gotStableMs = false;
        }
        interrupts();

        // snapshot hat pitch immediately on receipt of road tilt
        // hatPitchAtTilt and tiltSnapTaken are file-scope so they
        // reset cleanly between runs without reflashing.
        if (haveRoadTilt && !tiltSnapTaken) {
            hatPitchAtTilt = pitch - baselinePitch;
            capturedCarPitch = cp;
            tiltSnapTaken = true;
            Serial.print("Car pitch received: ");
            Serial.print(cp, 2);
            Serial.println(" deg");
            Serial.print("Hat relative pitch: ");
            Serial.print(hatPitchAtTilt, 2);
            Serial.println(" deg");
        }

        if (haveStableMs && tiltSnapTaken) {
            runCount++;
            printResult(capturedCarPitch, hatPitchAtTilt, se);
            ledFlash(3, 80, 80);

            // reset per-run state — ready for next hill
            tiltSnapTaken = false;
            hatPitchAtTilt = 0.0f;
            capturedCarPitch = 0.0f;

            // clear any stale flags that may have arrived during printResult
            noInterrupts();
            gotRoadTilt = false;
            gotStableMs = false;
            interrupts();

            digitalWrite(LED_PIN, HIGH); // solid = ready for next hill
            Serial.print("Run ");
            Serial.print(runCount);
            Serial.println(" complete. Car operator: reflash car and drive next hill.");
            Serial.println("Hat is ready — no reflash needed.");
        }

        // live monitor every 10 samples while waiting
        static int dbg = 0;
        if (++dbg >= 10) {
            dbg = 0;
            float rel = pitch - baselinePitch;
            Serial.print("rel=");
            Serial.print(rel, 1);
            Serial.print("°  rate=");
            Serial.print(pitchRate, 1);
            Serial.println("°/s  waiting for car...");
        }
    }
}
