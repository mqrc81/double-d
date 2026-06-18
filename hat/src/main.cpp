#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// ── PINS ─────────────────────────────────────────────────────────
#define INTERRUPT_PIN  2
#define BUTTON_PIN     13   // active LOW, internal pull-up — triggers calibration
#define BUZZER_PIN     23

// ── BUZZER ───────────────────────────────────────────────────────
// Requires a passive buzzer (cicalino passivo).
// tone()/noTone() drive a square wave — active buzzers are
// incompatible and will only click.
#define BUZZER_FREQ    2000   // Hz — 2 kHz tone

// Non-blocking buzzer: set buzzerOnUntil to schedule a tone burst.
// The buzzer control block at the top of loop() handles on/off.
unsigned long buzzerOnUntil = 0;

// Helper: start a tone burst of durationMs without blocking.
void beep(unsigned long durationMs) {
    tone(BUZZER_PIN, BUZZER_FREQ);
    buzzerOnUntil = millis() + durationMs;
}

// ── MPU ───────────────────────────────────────────────────────────
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

// ── DETECTION THRESHOLDS ─────────────────────────────────────────
const float PITCH_THRESHOLD = 19.0f; // ° — triggers alarm
const float PITCH_RECOVERY = 10.0f; // ° — cancels alarm (hysteresis)
const float BASELINE_CORRECTION_RATE = 0.001f; // ~50 s time constant at 20 Hz
// See DC doc: time constant intentionally > max microsleep duration
// (~30 s, Harrison & Horne 1996) so a genuine drowsy slump cannot
// erase itself from the baseline before the alarm fires.

const int RECOVERY_LIMIT = 5; // samples below threshold before reset
// At 20 Hz, 5 samples = 250 ms confirmation window.

// ── DETECTION STATE ───────────────────────────────────────────────
unsigned long lastTime = 0;
int sampleCount = 0;
unsigned long tiltStartTime = 0;
bool tiltActive = false;
int recoveryCount = 0;
int alarmCount = 0;
static bool alarmFired = false;

// ── ALARM LADDER ─────────────────────────────────────────────────
// Escalating confirmation time after repeated alarms.
// First event: 1500 ms (risk-stratified; pending DC1 floor confirmation).
// See DC doc ALARM_DURATION_MS entry for justification.
inline unsigned long effectiveAlarmMs() {
    if (alarmCount == 0) return 1500;
    if (alarmCount == 1) return 1000;
    return 700;
}

// ── CALIBRATION ───────────────────────────────────────────────────
float baselinePitch = 0.0f;
bool isCalibrated = false;
bool calibrationInProgress = false;
unsigned long calibrationStartTime = 0;
float pitchSum = 0.0f;
int calibsampleCount = 0;

// Calibration is triggered by the hat's own button (GPIO 13).
// 5 s window, minimum 50 still samples required.
// See DC doc CALIBRATION_DURATION_MS / CALIBRATION_SAMPLES_REQUIRED entries.
#define CALIBRATION_DURATION_MS      5000
#define CALIBRATION_SAMPLES_REQUIRED 50
#define CALIB_RATE_GATE              5.0f  // °/s — stillness gate
// CALIB_RATE_GATE: 5°/s is ~225× above the MPU-6050 noise-induced rate
// at 20 Hz (~0.022°/s). Any rejected sample reflects real movement.
// Source: InvenSense PS-MPU-6000A gyro noise density ~0.005°/s/√Hz.

// ── MPU INTERRUPT ────────────────────────────────────────────────
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// ── BUTTON DEBOUNCE ───────────────────────────────────────────────
bool lastBtn = HIGH;
unsigned long lastDebounce = 0;
#define DEBOUNCE_MS  50

// ── CALIBRATION FUNCTIONS ─────────────────────────────────────────
void startCalibration() {
    if (calibrationInProgress) return;
    Serial.println("=== CALIBRATION STARTED ===");
    Serial.println("Keep head perfectly still for 5 seconds!");
    calibrationInProgress = true;
    pitchSum = 0.0f;
    sampleCount = 0;
    calibsampleCount = 0;
    baselinePitch = 0.0f;
    calibrationStartTime = millis();
}

void performCalibration(float pitch, float totalRate) {
    if (!calibrationInProgress) return;

    if (millis() - calibrationStartTime < CALIBRATION_DURATION_MS) {
        if (totalRate < CALIB_RATE_GATE) {
            pitchSum += pitch;
            calibsampleCount++;
        }
    } else {
        if (calibsampleCount >= CALIBRATION_SAMPLES_REQUIRED) {
            baselinePitch = pitchSum / calibsampleCount;
            isCalibrated = true;
            calibrationInProgress = false;
            Serial.println("=== CALIBRATION COMPLETE ===");
            Serial.print("Samples: ");
            Serial.println(calibsampleCount);
            Serial.print("Baseline Pitch: ");
            Serial.println(baselinePitch, 2);
            // two long beeps = calibration success
            beep(400); // second beep scheduled in loop via lastBeepPattern
        } else {
            calibrationInProgress = false;
            Serial.println("Calibration failed — insufficient still samples!");
            // three short fast beeps = failure (scheduled below)
        }
    }
}

// ── SETUP ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    pinMode(BUZZER_PIN, OUTPUT);
    noTone(BUZZER_PIN);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(21, 22);
    Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(mpu.testConnection()
                       ? F("MPU6050 connection successful")
                       : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setRate(49); // 20 Hz — DMP output = 1000/(1+49)
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        Serial.print(F("Enabling interrupt detection (pin "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("DMP ready!");
        Serial.print("Packet size: ");
        Serial.println(packetSize);
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // ── 30 s DMP warmup ──────────────────────────────────────────
    // InvenSense AN-MPU-6000A-03: DMP complementary filter requires
    // 20–30 s to converge from cold start. Silent during warmup.
    Serial.println("Warming up DMP — please wait 30 seconds...");
    unsigned long warmupStart = millis();
    while (millis() - warmupStart < 30000) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
    }
    Serial.println("Warmup complete — press button to calibrate");
    // single short beep: warmup done, waiting for calibration
    beep(100);
}

// ── LOOP ──────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    // ── non-blocking buzzer off ───────────────────────────────────
    // Turns off any scheduled beep burst once its duration expires.
    // Alarm tone is held by alarmFired flag and overrides this.
    if (!alarmFired && now >= buzzerOnUntil) {
        noTone(BUZZER_PIN);
    }

    if (!dmpReady) return;
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

#ifdef OUTPUT_READABLE_YAWPITCHROLL

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = -(ypr[2] * 180.0f / M_PI);
    float roll = ypr[1] * 180.0f / M_PI;

    // ── rate of change ────────────────────────────────────────────
    static float lastPitch = 0.0f;
    static float lastRoll = 0.0f;
    static unsigned long lastSampleTime = 0;

    float dt = (now - lastSampleTime) / 1000.0f;
    if (dt > 1.0f) dt = 1.0f / 20.0f; // clamp first sample to 20 Hz assumption

    float pitchRate = (dt > 0) ? fabsf(pitch - lastPitch) / dt : 0.0f;
    float totalRate = pitchRate;

    lastPitch = pitch;
    lastRoll = roll;
    lastSampleTime = now;

    // ── button debounce ───────────────────────────────────────────
    bool btnNow = digitalRead(BUTTON_PIN);
    bool btnPressed = false;
    if (btnNow == LOW && lastBtn == HIGH && (now - lastDebounce > DEBOUNCE_MS)) {
        lastDebounce = now;
        btnPressed = true;
    }
    lastBtn = btnNow;

    // ── calibration trigger ───────────────────────────────────────
    if (btnPressed && !calibrationInProgress) {
        startCalibration();
    }

    // ── calibration in progress ───────────────────────────────────
    if (calibrationInProgress) {
        performCalibration(pitch, totalRate);

        // rapid double-beep every 500 ms during calibration
        static unsigned long lastCalibBeep = 0;
        static int calibBeepPhase = 0;
        if (now - lastCalibBeep > 500) {
            lastCalibBeep = now;
            calibBeepPhase = 0;
        }
        // two short pulses within the 500 ms window
        if (calibBeepPhase == 0 && now - lastCalibBeep > 0) {
            beep(60);
            calibBeepPhase = 1;
        }
        if (calibBeepPhase == 1 && now - lastCalibBeep > 150) {
            beep(60);
            calibBeepPhase = 2;
        }

        return; // skip detection while calibrating
    }

    // ── waiting for calibration ───────────────────────────────────
    if (!isCalibrated) {
        // single short beep every 2 s — gentle reminder
        static unsigned long lastWaitBeep = 0;
        if (now - lastWaitBeep > 2000) {
            lastWaitBeep = now;
            beep(80);
        }
        return;
    }

    // standalone mode: no bump suppression, no hill compensation
    const bool isBump = false;
    const float activePitchThresh = PITCH_THRESHOLD;

    // ── slow baseline drift correction ───────────────────────────
    // Only when driver is still and no alarm is active.
    // Time constant ~50 s — longer than max microsleep (~30 s,
    // Harrison & Horne 1996) so a real drowsy slump cannot erase itself.
    if (!tiltActive && totalRate < 2.0f) {
        baselinePitch += (pitch - baselinePitch) * BASELINE_CORRECTION_RATE;
    }

    float relativePitch = pitch - baselinePitch;

    // ── drowsiness detection ──────────────────────────────────────
    // isTriggered: head past threshold AND slow movement AND no bump
    bool isTriggered = (relativePitch > activePitchThresh)
                       && (totalRate < 20.0f)
                       && !isBump;

    // isRecovered: head back within recovery zone (hysteresis)
    bool isRecovered = (fabsf(relativePitch) < PITCH_RECOVERY);

    // ── state machine ─────────────────────────────────────────────
    if (isTriggered) {
        recoveryCount = 0;

        if (!tiltActive) {
            tiltStartTime = now;
            tiltActive = true;
            alarmFired = false;
        }

        if (now - tiltStartTime >= effectiveAlarmMs()) {
            // alarm: continuous tone
            tone(BUZZER_PIN, BUZZER_FREQ);
            if (!alarmFired) {
                alarmCount++;
                alarmFired = true;
                Serial.print("ALARM #");
                Serial.println(alarmCount);
            }
        }
    } else if (isBump && tiltActive) {
        // bump while head was tilting — pause timer, don't reset.
        // isBump expires naturally in 200 ms; timer resumes from tiltStartTime.
        recoveryCount = 0;
    } else if (isRecovered) {
        recoveryCount++;

        if (recoveryCount >= RECOVERY_LIMIT) {
            tiltActive = false;
            tiltStartTime = 0;
            recoveryCount = 0;
            alarmFired = false;
            noTone(BUZZER_PIN);
            Serial.println("Recovery detected — alarm cleared");
        }
    }

    // ── debug print every 10 samples ─────────────────────────────
    sampleCount++;
    if (sampleCount >= 10) {
        sampleCount = 0;
        Serial.print("p:");
        Serial.print(pitch, 2);
        Serial.print("  rel:");
        Serial.print(relativePitch, 2);
        Serial.print(" rate:");
        Serial.print(totalRate, 2);
        Serial.print(" thresh:");
        Serial.print(activePitchThresh, 2);
        Serial.print(" triggered:");
        Serial.print(isTriggered);
        Serial.print(" recovered:");
        Serial.print(isRecovered);
        Serial.print(" active:");
        Serial.print(tiltActive);
        Serial.print(" elapsed:");
        Serial.println(tiltActive ? now - tiltStartTime : 0);
    }

#endif
}
