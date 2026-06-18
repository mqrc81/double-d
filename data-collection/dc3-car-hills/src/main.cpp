// ================================================================
// DC3 — Hill Driving  (CAR UNIT)
// Car unit — ESP32-DevKitM-1 + MPU-6050
//
// PURPOSE
//   Detect road gradient entry, wait for pitch to stabilise, then
//   send two sequential ESP-NOW messages to the hat:
//     1. EVENT_ROAD_TILT  — settled car pitch (°)
//     2. EVENT_STABLE_MS  — elapsed ms from gradient entry to stable
//   The hat receives both, reads its own relative pitch, and logs
//   the full result. Car halts after sending.
//
//   Derives (logged by hat):
//     - ROAD_STABLE_MS    P80 of stableElapsed across all runs
//     - hillOffset factor mean of (hat_relPitch / car_pitch)
//
// HARDWARE  (identical to car production unit)
//   MPU-6050  on I2C default pins
//   Button    GPIO 26  (active LOW, internal pull-up)
//   LED       GPIO 13
//   INTERRUPT GPIO 4
//
// ESP-NOW
//   Car → hat only. No receive. No peer reply.
//   !! UPDATE hatMAC below to hat unit's actual MAC !!
//   (hat DC3 firmware prints its MAC to Serial on boot)
//
// PROCEDURE
//   Hat calibrates on flat ground first (see hat DC3 procedure).
//   Passenger watches hat LED — solid ON = hat ready.
//
//   1. Power on car unit. Wait 30 s DMP warmup (LED slow-blinks).
//   2. Car unit prints its MAC to Serial — note it for hat firmware.
//   3. Once hat LED is solid: drive toward first hill.
//   4. Press button as the car begins to climb or descend.
//      LED fast-blinks = capture armed.
//   5. Drive at steady speed onto the gradient.
//      Firmware detects pitch > MIN_HILL_DEG, waits for
//      totalRate < STABLE_RATE_GATE for STABLE_CONFIRM_MS.
//   6. On settle: sends EVENT_ROAD_TILT + EVENT_STABLE_MS to hat.
//      LED triple-flashes, then returns to slow-blink.
//      Car is ready for next hill — press button again.
//   7. Repeat from step 4 for each subsequent hill.
//   8. All results logged on hat Serial.
//
// NO REFLASH NEEDED BETWEEN HILLS.
// ================================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// ── PINS  (matches car production firmware exactly) ───────────────
#define INTERRUPT_PIN   4
#define BUTTON_PIN      26
#define LED_PIN         13

// ── SAMPLE RATE ───────────────────────────────────────────────────
#define SAMPLE_RATE_HZ  20   // mpu.setRate(49) → 1000/(1+49) = 20 Hz

// ── HILL DETECTION CONFIG ─────────────────────────────────────────
// Minimum absolute pitch to qualify as a real hill.
// Set just above DMP static noise floor (~1-2° per InvenSense PS-MPU-6000A).
// Replaces the removed ROAD_TILT_THRESH hard floor.
#define MIN_HILL_DEG        2.0f    // °

// Rate gate: car settled when totalRate stays below this.
// Matches production stabilisation gate (pending DC2 confirmation).
#define STABLE_RATE_GATE    8.0f    // °/s

// How long totalRate must hold below STABLE_RATE_GATE to confirm
// settlement. Separate from ROAD_STABLE_MS — this is a hysteresis
// guard, not the value being measured.
#define STABLE_CONFIRM_MS   500UL   // ms

// Abort if pitch never stabilises within this window.
#define STABLE_TIMEOUT_MS  8000UL   // ms

// ── HAT MAC ADDRESS ───────────────────────────────────────────────
// !! Replace with actual hat MAC before flashing !!
// Hat DC3 firmware prints its MAC to Serial on boot.
uint8_t hatMAC[] = {0xB0, 0xCB, 0xD8, 0x8B, 0x53, 0xC4};

// ── MESSAGE STRUCT  (identical to production) ─────────────────────
typedef struct {
    uint8_t eventType;
    float magnitude;
} ShoulderMessage;

#define EVENT_BUTTON     1
#define EVENT_BUMP       2
#define EVENT_ROAD_TILT  3
#define EVENT_STABLE_MS  4   // DC3 only: car sends stableElapsed to hat

ShoulderMessage msgOut;
esp_now_peer_info_t peerInfo;

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

// ── STATE ─────────────────────────────────────────────────────────
int runCount = 0;
bool hillArmed = false;
bool hillActive = false;
unsigned long gradientEntry = 0;
unsigned long stableStart = 0;
bool stableStarted = false;

// ── BUTTON DEBOUNCE ───────────────────────────────────────────────
bool lastBtn = HIGH;
unsigned long lastDebounce = 0;
#define DEBOUNCE_MS  50

// ── ESP-NOW SEND CALLBACK ─────────────────────────────────────────
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send OK" : "Send FAILED");
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

void sendResult(float settledPitch, unsigned long stableElapsed) {
    runCount++;

    // Message 1: settled car pitch
    msgOut.eventType = EVENT_ROAD_TILT;
    msgOut.magnitude = settledPitch;
    esp_now_send(hatMAC, (uint8_t *) &msgOut, sizeof(msgOut));
    delay(20); // brief gap so hat processes first message before second

    // Message 2: stabilisation elapsed time
    msgOut.eventType = EVENT_STABLE_MS;
    msgOut.magnitude = (float) stableElapsed;
    esp_now_send(hatMAC, (uint8_t *) &msgOut, sizeof(msgOut));

    Serial.print("Run ");
    Serial.print(runCount);
    Serial.print(" sent: car_pitch=");
    Serial.print(settledPitch, 2);
    Serial.print(" deg  stable_elapsed=");
    Serial.print(stableElapsed);
    Serial.println(" ms");

    ledFlash(3, 80, 80);

    // reset per-run state — ready for next button press
    hillArmed = false;
    hillActive = false;
    gradientEntry = 0;
    stableStart = 0;
    stableStarted = false;

    digitalWrite(LED_PIN, LOW); // slow-blink resumes in loop
    Serial.println("Ready for next hill — press button on next gradient onset.");
}

// ── SETUP ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    Wire.begin();
    Wire.setClock(400000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.print("Car MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init FAILED");
        while (true) { ledFlash(1, 100, 100); }
    }
    esp_now_register_send_cb(onSent);
    // no recv callback — car does not receive in DC3

    memcpy(peerInfo.peer_addr, hatMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add hat peer");
        while (true) { ledFlash(1, 100, 100); }
    }

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

    Serial.println("DMP warmup 30 s — drive toward hill.");
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
    Serial.println("Ready. Confirm hat LED solid, then press button at each gradient onset.");
}

// ── LOOP ──────────────────────────────────────────────────────────
void loop() {
    if (!dmpReady) return;

    unsigned long now = millis();

    // ── button ───────────────────────────────────────────────────
    bool btnNow = digitalRead(BUTTON_PIN);
    if (btnNow == LOW && lastBtn == HIGH && (now - lastDebounce > DEBOUNCE_MS)) {
        lastDebounce = now;
        if (!hillArmed) {
            hillArmed = true;
            hillActive = false;
            gradientEntry = 0;
            stableStart = 0;
            stableStarted = false;
            Serial.println("Hill capture ARMED.");
        }
    }
    lastBtn = btnNow;

    // ── sensor read ───────────────────────────────────────────────
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

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
    float totalRate = max(pitchRate, rollRate);
    lastPitch = pitch;
    lastRoll = roll;
    lastSampleTime = now;

    if (!hillArmed) {
        digitalWrite(LED_PIN, (now / 500) % 2);
        static int dbg = 0;
        if (++dbg >= 10) {
            dbg = 0;
            Serial.print("p:");
            Serial.print(pitch, 1);
            Serial.print(" rate:");
            Serial.println(totalRate, 1);
        }
        return;
    }

    // ── armed: detect gradient entry ─────────────────────────────
    digitalWrite(LED_PIN, (now / 150) % 2);

    if (!hillActive) {
        if (fabsf(pitch) > MIN_HILL_DEG) {
            hillActive = true;
            gradientEntry = now;
            Serial.print("Gradient entry: pitch=");
            Serial.print(pitch, 2);
            Serial.println("° — waiting to stabilise...");
        }
        return;
    }

    // ── hill active: watch for abort conditions ───────────────────
    if (fabsf(pitch) < MIN_HILL_DEG) {
        Serial.println("Pitch lost before settling — run aborted. Press button on next onset.");
        hillArmed = false;
        hillActive = false;
        gradientEntry = 0;
        stableStart = 0;
        stableStarted = false;
        digitalWrite(LED_PIN, LOW);
        return;
    }

    if ((now - gradientEntry) > STABLE_TIMEOUT_MS) {
        Serial.println("Stabilisation timeout — run aborted. Press button on next onset.");
        hillArmed = false;
        hillActive = false;
        gradientEntry = 0;
        stableStart = 0;
        stableStarted = false;
        digitalWrite(LED_PIN, LOW);
        return;
    }

    // ── track stable window ───────────────────────────────────────
    if (totalRate < STABLE_RATE_GATE) {
        if (!stableStarted) {
            stableStart = now;
            stableStarted = true;
        }
        if ((now - stableStart) >= STABLE_CONFIRM_MS) {
            // stableElapsed: gradient entry → first stable sample
            // this is the candidate ROAD_STABLE_MS value
            unsigned long stableElapsed = stableStart - gradientEntry;
            sendResult(pitch, stableElapsed);
        }
    } else {
        stableStarted = false;
    }

    static int dbg2 = 0;
    if (++dbg2 >= 10) {
        dbg2 = 0;
        Serial.print("p:");
        Serial.print(pitch, 1);
        Serial.print(" rate:");
        Serial.print(totalRate, 1);
        Serial.print(" stable:");
        Serial.println(stableStarted ? "yes" : "no");
    }
}
