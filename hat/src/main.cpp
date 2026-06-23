#include "I2Cdev.h"
#include <esp_now.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "WiFi.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  
#define BUZZER_PIN    13 
#define BUZZER_FREQ   2000
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
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int sampleCount = 0;
unsigned long tiltStartTime = 0;
bool tiltActive = false;
int   recoveryCount  = 0;
int  alarmCount    = 0;
volatile bool alarmActive = false;
const int   RECOVERY_LIMIT = 5;        // samples below threshold before reset
const float PITCH_THRESHOLD  = 4.0;   // must exceed this to start alarm
const float PITCH_RECOVERY   = 3.0;   // only needs to drop below this to cancel
const float BASELINE_CORRECTION_RATE = 0.001f;

typedef struct {
    uint8_t eventType;
    float   magnitude;
} ShoulderMessage;
uint8_t broadcastadd[] = {0x20, 0xE7, 0xC8, 0xBB, 0x16, 0xC0};

#define EVENT_BUTTON    1
#define EVENT_BUMP      2
#define EVENT_ROAD_TILT 3

ShoulderMessage msgIn;

volatile bool          newCalibRequest = false;
volatile unsigned long bumpTimestamp   = 0xFFFFFFFF;
volatile float         roadAngle       = 0;

// INTERRUPT DETECTION ROUTINE

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void buzzerTask(void* pvParameters) {
    pinMode(BUZZER_PIN, OUTPUT);
    bool lastState = false;

    for (;;) {
        bool current = alarmActive;
        if (current && !lastState) {
            tone(BUZZER_PIN, BUZZER_FREQ);
        } else if (!current && lastState) {
            noTone(BUZZER_PIN);
        }
        lastState = current;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
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

// INITIAL SETUP


void setup() {
    Serial.begin(115200);

    xTaskCreatePinnedToCore(
        buzzerTask, "buzzerTask", 1024, NULL, 1, NULL, 0
    );

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    WiFi.mode(WIFI_MODE_STA);
    if (esp_now_init() != ESP_OK){
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    Serial.print("MAC Address: ");
    Serial.print(WiFi.macAddress());
    Serial.print("\n");

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

    // making sur it worked
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);

        mpu.setRate(49); // sampling rate set to 20Hz
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

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
    // Register the receive callback
    esp_now_register_recv_cb(onDataReceived);

    Serial.println("Warming up DMP — please wait 30 seconds...");
    unsigned long warmupStart = millis();
    while (millis() - warmupStart < 30000) {
        // keep reading packets so DMP filter converges
        // change it to 10 when testing because it is taking to long
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
        delay(1); // this is for the wire error
    }
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
    baselinePitch        = 0;
    calibrationStartTime = millis();
    alarmCount = 0;
}

void performCalibration(float pitch, float totalRate) {
    if (!calibrationInProgress) return;

    // Collect samples for the calibration window
    if (millis() - calibrationStartTime < CALIBRATION_DURATION_MS) {
        if (totalRate < 5.0f) {
            pitchSum += pitch;
            calibsampleCount++;
        }
    }
    else {
        // Calibration complete
        if (calibsampleCount >= CALIBRATION_SAMPLES_REQUIRED) {
            baselinePitch = pitchSum / calibsampleCount;
            isCalibrated = true;

            Serial.println("=== CALIBRATION COMPLETE ===");
            Serial.print("Samples: ");
            Serial.println(calibsampleCount);
            Serial.print("Baseline Pitch: ");
            Serial.println(baselinePitch);
            alarmActive = true; 
            delay(200);
            alarmActive = false; 
            delay(150);
            alarmActive = true;  
            delay(200);
            alarmActive = false;
        } else {
            Serial.println("Calibration failed - insufficient samples!");
        }

        calibrationInProgress = false;
    }
}

// MAIN PROGRAM LOOP

void loop() {
    if (!dmpReady) return;

    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

    #ifdef OUTPUT_READABLE_YAWPITCHROLL

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // depends on the sensor position
        float pitch = -(ypr[2] * 180/M_PI);

        
        unsigned long now = millis();

        // rate of change 
        static float         lastPitch      = 0;
        static unsigned long lastSampleTime = 0;
        static bool          alarmFired     = false;

        float dt = (now - lastSampleTime) / 1000.0f;
        if (dt > 1.0f) dt = 0.05f;   // clamp first sample

        float totalRate = (dt > 0) ? abs(pitch - lastPitch) / dt : 0;

        lastPitch      = pitch;
        lastSampleTime = now;

        // handle shoulder button press
        // read and immediately clear the flag
        // avoids acting on it twice
        bool doCalibrate = false;
        if (newCalibRequest) {
            newCalibRequest = false;
            doCalibrate     = true;
        }

        // calibration state machine 
        if (doCalibrate && !calibrationInProgress) {
            startCalibration();
        }

    if (calibrationInProgress) {
        performCalibration(pitch, totalRate);
        // only blink during sampling 
        if (calibrationInProgress) {
            unsigned long elapsed = millis() - calibrationStartTime;
            unsigned long phase   = elapsed % 500;
            alarmActive = (phase < 60) || (phase >= 150 && phase < 210);
        }
    return;
    }

        if (!isCalibrated) {
            alarmActive = false; // stay silent while waiting for button
            return;  
        }
        unsigned long effectiveAlarmMs;// alarmed state after first detection
        if      (alarmCount == 0) effectiveAlarmMs = 800;
        else if (alarmCount == 1) effectiveAlarmMs = 500;
        else                      effectiveAlarmMs = 200;

        // ── shoulder data processing 

        // bump — true if shoulder reported a jolt within last 200ms
        bool isBump = (now - bumpTimestamp) < 200;

        // road tilt — adjust pitch threshold based on hill angle
        // on a hill drivers body tilts with the car
        // so we loosen threshold proportionally
        float hillOffset = roadAngle;

        if (roadAngle < 0) {
            // uphill easier to false trigger
            hillOffset = max(roadAngle * 0.7f, -15.0f);
        } else {
            // downhill — body leans backward, harder to genuinely drop forward
            // slight tightening, capped so it doesn't become too aggressive
            hillOffset = min(roadAngle *0.7f, 15.0f);
        }
        float activePitchThresh = PITCH_THRESHOLD + hillOffset;

        // relative angles from calibrated baseline 
        // slowly correct baseline to follow DMP drift
        // only when driver is still and not in alarm
        if (!tiltActive && totalRate < 2.0f) {
            baselinePitch += (pitch - baselinePitch) * BASELINE_CORRECTION_RATE;
        }

        float relativePitch = pitch - baselinePitch;

        // drowsiness detection 

        // isTriggered conditions:
        // 1. head is past threshold
        // 2. movement is slow (not a shake or bump recovery)
        // 3. no bump reported from shoulder in last 200ms
        bool isTriggered = (relativePitch > activePitchThresh)
                           && (totalRate < 20.0f)
                           && !isBump;

        // isRecovered — looser threshold to avoid LED flickering
        // on the boundary (hysteresis)
        bool isRecovered = (abs(relativePitch) < PITCH_RECOVERY);

        if (isTriggered) {
            recoveryCount = 0;

            if (!tiltActive) {
                tiltStartTime = now;
                tiltActive    = true;
                alarmFired    = false;
                Serial.println("Threshold crossed");
            }

            // alarm only after sustained tilt
            if (now - tiltStartTime >= effectiveAlarmMs) {
                alarmActive = true;
                if (!alarmFired) {
                    alarmCount++;        // count this alarm event
                    alarmFired = true;
                    Serial.print("ALARM #");
                    Serial.println(alarmCount);
                    Serial.print("Time from threshold to alarm: ");
                    Serial.print(now - tiltStartTime);
                    Serial.println(" ms");
                }
            }

        } else if (isBump && tiltActive) {
            // bump arrived while head was tilting
            // pause the timer — don't reset it fully
            // because driver might still be drowsy after bump
            // just give them benefit of the doubt for 200ms
            // timer resumes when isBump expires naturally
            recoveryCount = 0;

        } else if (isRecovered) {
            recoveryCount++;

            if (recoveryCount >= RECOVERY_LIMIT) {
                // genuine recovery — head is up
                tiltActive    = false;
                tiltStartTime = 0;
                recoveryCount = 0;
                alarmFired    = false;
                alarmActive   = false;
            }
        }
        // between recovery and trigger threshold it holds state
        // this prevents flickering on the boundary

        //  debug print every 10 samples
        sampleCount++;
        if (sampleCount >= 10) {
            sampleCount = 0;

            Serial.print("RAW p:"); Serial.print(pitch);
            Serial.print("  p:");   Serial.print(relativePitch);
            Serial.print(" rate:");    Serial.print(totalRate);
            Serial.print(" bump:");    Serial.print(isBump);
            Serial.print(" hill:");    Serial.print(roadAngle);
            Serial.print(" thresh:");  Serial.print(activePitchThresh);
            Serial.print(" triggered:"); Serial.print(isTriggered);
            Serial.print(" recovered:"); Serial.print(isRecovered);
            Serial.print(" active:");  Serial.print(tiltActive);
            Serial.print(" elapsed:");
            Serial.println(tiltActive ? now - tiltStartTime : 0);
        }

    #endif
}
