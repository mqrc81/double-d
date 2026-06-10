#include "I2Cdev.h"
#include <esp_now.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "WiFi.h"
#include <Adafruit_INA219.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

//Consumption reading
Adafruit_INA219 ina219;
float totalEnergyMWh = 0;
unsigned long lastEnergyMs = 0;
unsigned long lastPrintMs = 0;
// add with other globals at top
#define TELEPLOT_MODE true   // true = teleplot graphs, false = debug text

// Calibration Variables
float baselinePitch = 0;
bool isCalibrated = false;
bool calibrationInProgress = false;

unsigned long calibrationStartTime = 0;
#define CALIBRATION_DURATION_MS 5000   // 5 seconds of sampling
#define CALIBRATION_SAMPLES_REQUIRED 50

float pitchSum = 0;
int calibsampleCount = 0;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q; // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

unsigned long lastTime = 0;
int sampleCount = 0;
unsigned long tiltStartTime = 0;
bool tiltActive = false;
int recoveryCount = 0;
int alarmCount = 0;
const int RECOVERY_LIMIT = 5; // samples below threshold before reset
unsigned long ALARM_DURATION_MS = 1000; // 3 real seconds
const float PITCH_THRESHOLD = 19.0; // must exceed this to start alarm
const float PITCH_RECOVERY = 10.0; // only needs to drop below this to cancel
const float BASELINE_CORRECTION_RATE = 0.001f;

//esp32 info
uint8_t broadcastadd[] = {0x20, 0xE7, 0xC8, 0xBB, 0x16, 0xC0};

typedef struct {
    uint8_t eventType;
    float magnitude;
} ShoulderMessage;

#define EVENT_BUTTON    1
#define EVENT_BUMP      2
#define EVENT_ROAD_TILT 3

ShoulderMessage msgIn;

volatile bool newCalibRequest = false;
volatile unsigned long bumpTimestamp = 0xFFFFFFFF;
volatile float roadAngle = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void onDataReceived(const uint8_t *mac,
                    const uint8_t *data, int len) {
    if (len != sizeof(ShoulderMessage)) return;
    memcpy(&msgIn, data, sizeof(ShoulderMessage));

    switch (msgIn.eventType) {
        case EVENT_BUTTON:
            newCalibRequest = true;
            break;
        case EVENT_BUMP:
            bumpTimestamp = millis();
            break;
        case EVENT_ROAD_TILT:
            roadAngle = msgIn.magnitude;
            break;
    }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================


void setup() {
    Serial.begin(115200);
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    if (!ina219.begin()) {
        Serial.println("INA219 not found — check wiring");
    } else {
        Serial.println("INA219 ready");
        lastEnergyMs = millis();
    }
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)

    WiFi.mode(WIFI_MODE_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("errpr initialiying esp-now");
        return;
    }
    Serial.print("MAC Address: ");
    Serial.print(WiFi.macAddress());
    Serial.print("\n");

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);

        mpu.setRate(49); // sampling rate set to 20Hz
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // After setDMPEnabled(true), add:
        mpuIntStatus = mpu.getIntStatus();
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);


        // get our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("DMP ready!");
        Serial.print("Packet size: ");
        Serial.println(packetSize);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    // Register the send callback
    esp_now_register_recv_cb(onDataReceived);

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    Serial.println("Warming up DMP — please wait 30 seconds...");
    unsigned long warmupStart = millis();
    while (millis() - warmupStart < 30000) {
        // keep reading packets so DMP filter converges
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
        // blink LED slowly during warmup
        digitalWrite(LED_PIN, (millis() / 500) % 2);
    }
    digitalWrite(LED_PIN, LOW);
    Serial.println("Warmup complete — press button to calibrate");
}

void startCalibration() {
    if (calibrationInProgress) {
        Serial.println("Calibration already in progress");
        return;
    }

    Serial.println("=== CALIBRATION STARTED ===");
    Serial.println("Keep head perfectly still for 5 seconds!");

    // Reset calibration variables
    calibrationInProgress = true;
    pitchSum = 0;
    sampleCount = 0;
    calibsampleCount = 0;
    baselinePitch = 0;
    calibrationStartTime = millis();
}

void performCalibration(float pitch, float roll, float totalRate) {
    if (!calibrationInProgress) return;

    // Collect samples for 3 seconds
    if (millis() - calibrationStartTime < CALIBRATION_DURATION_MS) {
        if (totalRate < 5.0f) {
            pitchSum += pitch;
            calibsampleCount++;

            // Blink LED during sampling
            //digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
            //delay(20);
        }
    } else {
        // Calibration complete
        if (calibsampleCount >= CALIBRATION_SAMPLES_REQUIRED) {
            baselinePitch = pitchSum / calibsampleCount;
            isCalibrated = true;

            Serial.println("=== CALIBRATION COMPLETE ===");
            Serial.print("Samples: ");
            Serial.println(calibsampleCount);
            Serial.print("Baseline Pitch: ");
            Serial.print(baselinePitch);

            // Success pattern: 3 long blinks
            /* for (int i = 0; i < 3; i++) {
                 digitalWrite(STATUS_LED_PIN, HIGH);
                 delay(300);
                 digitalWrite(STATUS_LED_PIN, LOW);
                 delay(200);
             }*/
        } else {
            Serial.println("Calibration failed - insufficient samples!");
            /*for (int i = 0; i < 5; i++) {
                digitalWrite(STATUS_LED_PIN, HIGH);
                delay(100);
                digitalWrite(STATUS_LED_PIN, LOW);
                delay(100);
            }*/
        }

        calibrationInProgress = false;
        //digitalWrite(STATUS_LED_PIN, LOW);
    }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // ── INA219 power measurement ──────────────────────────────
    float current_mA = ina219.getCurrent_mA();
    float voltage_V = ina219.getBusVoltage_V();
    float power_mW = ina219.getPower_mW();

    unsigned long nowEnergy = millis();
    float dt_hours = (nowEnergy - lastEnergyMs) / 3600000.0f;
    totalEnergyMWh += power_mW * dt_hours;
    lastEnergyMs = nowEnergy;

    if ((nowEnergy - lastPrintMs > 200) && (TELEPLOT_MODE)) {
        lastPrintMs = nowEnergy;
        Serial.print(">current:");
        Serial.println(current_mA);
        Serial.print(">voltage:");
        Serial.println(voltage_V);
        Serial.print(">power:");
        Serial.println(power_mW);
        Serial.print(">energy:");
        Serial.println(totalEnergyMWh);
    }
    if (!dmpReady) return;

    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

#ifdef OUTPUT_READABLE_YAWPITCHROLL

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // adjust ypr indices if your sensor is mounted differently
    float pitch = -(ypr[2] * 180 / M_PI);
    float roll = ypr[1] * 180 / M_PI;


    // ── single timestamp for entire loop ──────────────────
    unsigned long now = millis();

    // ── rate of change ────────────────────────────────────
    static float lastPitch = 0;
    static float lastRoll = 0;
    static unsigned long lastSampleTime = 0;
    static bool alarmFired = false;

    float dt = (now - lastSampleTime) / 1000.0f;
    if (dt > 1.0f) dt = 0.05f; // clamp first sample

    float pitchRate = (dt > 0) ? abs(pitch - lastPitch) / dt : 0;
    float totalRate = pitchRate;

    lastPitch = pitch;
    lastRoll = roll;
    lastSampleTime = now;

    // ── handle shoulder button press ──────────────────────
    // read and immediately clear the flag
    // avoids acting on it twice
    bool doCalibrate = false;
    if (newCalibRequest) {
        newCalibRequest = false;
        doCalibrate = true;
    }

    // ── calibration state machine ─────────────────────────
    if (doCalibrate && !calibrationInProgress) {
        startCalibration();
    }

    if (calibrationInProgress) {
        performCalibration(pitch, roll, totalRate);
        // fast blink during calibration
        digitalWrite(LED_PIN, (now / 150) % 2);
        return; // skip detection while calibrating
    }

    if (!isCalibrated) {
        // waiting for button — slow blink
        digitalWrite(LED_PIN, (now / 500) % 2);
        return; // skip detection until calibrated
    }
    unsigned long effectiveAlarmMs;
    if (alarmCount == 0) effectiveAlarmMs = 1500;
    else if (alarmCount == 1) effectiveAlarmMs = 1000;
    else effectiveAlarmMs = 700;
    // ── shoulder data processing ──────────────────────────

    // bump — true if shoulder reported a jolt within last 200ms
    bool isBump = (now - bumpTimestamp) < 200;

    // road tilt — adjust pitch threshold based on hill angle
    // on a 10° hill, driver's body tilts ~5° with car
    // so we loosen threshold proportionally
    float hillOffset = min(abs(roadAngle) * 0.8f, 15.0f); // max 10° adjustment
    float activePitchThresh = PITCH_THRESHOLD + hillOffset;
    // ── relative angles from calibrated baseline ──────────
    // slowly correct baseline to follow DMP drift
    // only when driver is still and not in alarm
    if (!tiltActive && totalRate < 2.0f) {
        baselinePitch += (pitch - baselinePitch) * BASELINE_CORRECTION_RATE;
    }

    float relativePitch = pitch - baselinePitch;
    // ── drowsiness detection ──────────────────────────────

    // isTriggered conditions:
    // 1. head is past threshold in either axis
    // 2. movement is slow (not a shake or bump recovery)
    // 3. no bump reported from shoulder in last 200ms
    bool isTriggered = (relativePitch > activePitchThresh)
                       && (totalRate < 20.0f)
                       && !isBump;

    // isRecovered — looser threshold to avoid LED flickering
    // on the boundary (hysteresis)
    bool isRecovered = (abs(relativePitch) < PITCH_RECOVERY);

    // ── state machine ─────────────────────────────────────
    if (isTriggered) {
        recoveryCount = 0;

        if (!tiltActive) {
            tiltStartTime = now;
            tiltActive = true;
            alarmFired = false;
        }

        // alarm only after sustained tilt
        if (now - tiltStartTime >= effectiveAlarmMs) {
            digitalWrite(LED_PIN, HIGH);
            if (!alarmFired) {
                alarmCount++; // count this alarm event
                alarmFired = true;
                Serial.print("ALARM #");
                Serial.println(alarmCount);
            }
        }
    } else if (isBump && tiltActive) {
        // bump arrived while head was tilting
        // pause the timer — don't reset it fully
        // because driver might still be drowsy after bump
        // just give them benefit of the doubt for 200ms
        // timer resumes when isBump expires naturally
        // (nothing to do here — isBump will be false
        //  in 200ms and timer continues from tiltStartTime)
        recoveryCount = 0;
    } else if (isRecovered) {
        recoveryCount++;

        if (recoveryCount >= RECOVERY_LIMIT) {
            // genuine recovery — head is up
            tiltActive = false;
            tiltStartTime = 0;
            recoveryCount = 0;
            alarmFired = false;
            digitalWrite(LED_PIN, LOW);
            // noTone(BUZZER_PIN);
        }
    }
    // between recovery and trigger threshold → hold state
    // this prevents flickering on the boundary

    // ── debug print every 10 samples ─────────────────────
    sampleCount++;
    if (sampleCount >= 10 && (!TELEPLOT_MODE)) {
        sampleCount = 0;

        Serial.print("RAW p:");
        Serial.print(pitch);
        Serial.print("  p:");
        Serial.print(relativePitch);
        Serial.print(" rate:");
        Serial.print(totalRate);
        Serial.print(" bump:");
        Serial.print(isBump);
        Serial.print(" hill:");
        Serial.print(roadAngle);
        Serial.print(" thresh:");
        Serial.print(activePitchThresh);

        // ADD THESE TWO LINES
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
