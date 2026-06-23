#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <esp_now.h>
#include <WiFi.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//  PINS 
#define INTERRUPT_PIN  4
#define BUTTON_PIN     26    // calibration button
#define LED_PIN        13     

unsigned long lastBumpSent = 0;
const unsigned long BUMP_COOLDOWN_MS = 500;  // one bump per 500ms max
// HEAD UNIT MAC  
uint8_t headMAC[] = {0x20,0xE7,0xC8,0xBB,0x16,0xC0};

// MESSAGE STRUCT 
typedef struct {
    uint8_t eventType;      // what happened 
    float   magnitude;      // bump strength OR road angle
} ShoulderMessage;

// event type values
#define EVENT_BUTTON    1   // driver pressed calibration button
#define EVENT_BUMP      2   // road bump detected
#define EVENT_ROAD_TILT 3   // uphill or downhill detected

ShoulderMessage msgOut;
esp_now_peer_info_t peerInfo;

unsigned long ledOnTime = 0;

// MPU6050 
MPU6050 mpu;
bool     dmpReady  = false;
uint8_t  devStatus;
uint16_t packetSize;
uint8_t  fifoBuffer[64];
Quaternion  q;
VectorFloat gravity;
float       ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// BUMP DETECTION 
const float BUMP_RATE_THRESH  = 80.0f;   // °/s — sudden jolt
const float ROAD_TILT_THRESH  = 4.0f;   // degrees — uphill/downhill
#define ROAD_STABLE_MS  500 // tilt must persist 1s



// road tilt state
float         roadTiltAngle    = 0;
unsigned long roadTiltStart    = 0;
bool          roadTiltActive   = false;
bool          roadTiltReported = false;

//  BUTTON DEBOUNCE 
bool     lastButtonState = HIGH;
unsigned long lastDebounce  = 0;

//  ESP-NOW SEND 
void sendEvent(uint8_t type, float magnitude) {
    msgOut.eventType = type;
    msgOut.magnitude = magnitude;
    esp_now_send(headMAC, (uint8_t*)&msgOut, sizeof(msgOut));

    // brief LED flash to confirm send
    digitalWrite(LED_PIN, HIGH);
    // non-blocking — just note the time, turn off next loop
    ledOnTime = millis();
    
}

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    // optional debug
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send FAIL");
}

// SETUP 
void setup() {
    Serial.begin(115200);
    delay(1000);  // give serial Monitor time to open

    // print MAC before WiFi mode is set
    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    Serial.print("Base MAC (STA): ");
    for (int i = 0; i < 6; i++) {
        if (baseMac[i] < 0x10) Serial.print("0");
        Serial.print(baseMac[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN,    OUTPUT);

    Wire.begin();
    Wire.setClock(400000);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

   
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    Serial.print("Base MAC (STA): ");
    for (int i = 0; i < 6; i++) {
        if (baseMac[i] < 0x10) Serial.print("0");
        Serial.print(baseMac[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    Serial.print("WiFi MAC: ");
    Serial.println(WiFi.macAddress());

    // initialisation of ESP-NOW
    esp_now_init();
    esp_now_register_send_cb(onSent);
    memcpy(peerInfo.peer_addr, headMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    // MPU6050
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(mpu.testConnection() ?
        "MPU6050 OK" : "MPU6050 FAILED");


    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setRate(49);          // 20Hz
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        dmpReady   = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("Shoulder unit ready");
    } else {
        Serial.print("DMP failed: ");
        Serial.println(devStatus);
    }

}


void loop() {
    if (!dmpReady) return;

    unsigned long now = millis();

    if (ledOnTime > 0 && (now - ledOnTime) > 100) {
    digitalWrite(LED_PIN, LOW);
    ledOnTime = 0;
    }
    static float         lastPitch      = 0;
    static float         lastRoll       = 0;
    static unsigned long lastSampleTime = 0;

    static bool firstSample = true;

    // button check 
    bool btnNow = digitalRead(BUTTON_PIN);
    if (btnNow == LOW && lastButtonState == HIGH
        && (now - lastDebounce > 50)) {
        lastDebounce = now;
        Serial.println("Button pressed — sending calibration trigger");
        sendEvent(EVENT_BUTTON, 0);
    }
    lastButtonState = btnNow;

    //  sensor read 
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = -(ypr[2] * 180/M_PI);
    float roll  = ypr[1] * 180/M_PI;

    if (firstSample) {
    firstSample    = false;
    lastPitch      = pitch;
    lastRoll       = roll;
    lastSampleTime = now;
    return;   // skip calculations on first sample
    }

    //  rate of change
    float dt = (now - lastSampleTime) / 1000.0f;
    if (dt > 1.0f) dt = 0.05f;
    float pitchRate = (dt > 0) ? abs(pitch - lastPitch) / dt : 0;
    float rollRate  = (dt > 0) ? abs(roll  - lastRoll)  / dt : 0;
    float totalRate = max(pitchRate, rollRate);
    lastPitch      = pitch;
    lastRoll       = roll;
    lastSampleTime = now;

    // bump detection 
    // sudden fast movement = bump
    if (totalRate > BUMP_RATE_THRESH) {
        if (now - lastBumpSent > BUMP_COOLDOWN_MS) {
        Serial.print("Bump! rate: ");
        Serial.println(totalRate);
        sendEvent(EVENT_BUMP, totalRate);
        lastBumpSent = now;
        }
    }

    // road tilt detection 
    // uphill/downhill = sustained pitch change on SHOULDER sensor
    // shoulder sensor sits flat in car so its pitch = road angle
    bool roadTilted = abs(pitch) > ROAD_TILT_THRESH;   // slow change = road, not bump

    if (roadTilted) {
    if (!roadTiltActive) {
        roadTiltActive   = true;
        roadTiltStart    = now;
        roadTiltReported = false;
        roadTiltAngle    = pitch;
        Serial.println("Hill starting — early warning sent");
    }

    // step 2  update the angle continuously while tilted
    if (!roadTiltReported) {
        roadTiltAngle = pitch;  // only update before first send
    }

    // step 3 — report after 300ms (reduced from 1000ms), because t took to long to send
    if (!roadTiltReported
        && (now - roadTiltStart >= 300)
        && totalRate < 8.0f) {
        Serial.print("Road tilt: ");
        Serial.println(roadTiltAngle);
        sendEvent(EVENT_ROAD_TILT, roadTiltAngle);
        roadTiltReported = true;
    }

    // step 4 — if angle changes significantly after reporting
    // send an update (car is still going up a steeper grade)
    if (roadTiltReported && abs(pitch - roadTiltAngle) > 3.0f) {
    roadTiltAngle    = pitch;
    Serial.print("Road tilt updated: ");
    Serial.println(roadTiltAngle);
    sendEvent(EVENT_ROAD_TILT, roadTiltAngle);  // ← send immediately
    // keep roadTiltReported = true so step 4 keeps monitoring
    roadTiltStart    = now;
    }  

    } else {
        if (roadTiltActive && roadTiltReported) {
            Serial.println("Road level again");
            sendEvent(EVENT_ROAD_TILT, 0);
        }
        roadTiltActive   = false;
        roadTiltReported = false;
    }

    // debug 
    static int dbgCount = 0;
    if (++dbgCount >= 10) {
        dbgCount = 0;
        Serial.print("p:"); Serial.print(pitch);
        Serial.print(" r:"); Serial.print(roll);
        Serial.print(" rate:"); Serial.println(totalRate);
    }
}
