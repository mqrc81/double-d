#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Preferences.h>

#define SDA_PIN 17
#define SCL_PIN 18
#define LED_PIN 35      // onboard LED for status feedback
#define BUTTON_PIN 0    // PRG button to trigger calibration

#define CALIBRATION_SECONDS     30
#define SAMPLE_RATE_HZ          50
#define TOTAL_SAMPLES           (CALIBRATION_SECONDS * SAMPLE_RATE_HZ)  // 1500

MPU6050 mpu;
Preferences prefs;

float pitch = 0.0f;
float roll = 0.0f;
const float ALPHA = 0.96f; // complementary filter coefficient
unsigned long lastTime = 0;

void updateAngle();

void runCalibration();

void printStoredProfile();

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, LOW);

    // Init I2C and MPU6050
    Wire.begin(SDA_PIN, SCL_PIN);
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("[ERROR] MPU6050 connection failed. Check wiring.");
        while (true) { delay(1000); }
    }
    Serial.println("[OK] MPU6050 connected.");

    // Print any existing stored profile
    printStoredProfile();

    Serial.println("\nPress PRG button to start calibration.");
    Serial.println("Sit still, look forward naturally for 30 seconds.");
}

// ─────────────────────────────────────────────────────────────────
void loop() {
    // Wait for button press to trigger calibration
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50); // debounce
        if (digitalRead(BUTTON_PIN) == LOW) {
            runCalibration();
        }
    }
}

void updateAngle() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long now = micros();
    float dt = (now - lastTime) / 1e6f;
    lastTime = now;

    // Accelerometer angle (degrees)
    float accelPitch = atan2f((float) ay, (float) az) * 180.0f / PI;
    float accelRoll = atan2f((float) ax, (float) az) * 180.0f / PI;

    // Gyroscope rate (degrees/sec) — MPU6050 default sensitivity 131 LSB/deg/s
    float gyroPitchRate = (float) gx / 131.0f;
    float gyroRollRate = (float) gy / 131.0f;

    // Complementary filter
    pitch = ALPHA * (pitch + gyroPitchRate * dt) + (1.0f - ALPHA) * accelPitch;
    roll = ALPHA * (roll + gyroRollRate * dt) + (1.0f - ALPHA) * accelRoll;
}

void runCalibration() {
    Serial.println("\n[CAL] Starting calibration in 3 seconds — sit still...");

    // Blink LED 3 times as countdown
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(400);
        digitalWrite(LED_PIN, LOW);
        delay(400);
    }

    Serial.println("[CAL] Collecting samples...");
    digitalWrite(LED_PIN, HIGH); // solid LED = calibrating

    // Reset filter state
    pitch = 0.0f;
    roll = 0.0f;
    lastTime = micros();

    // Welford's online algorithm for mean and variance
    // avoids storing all samples — memory safe on ESP32
    double meanPitch = 0, meanRoll = 0;
    double M2Pitch = 0, M2Roll = 0; // sum of squared deviations
    int count = 0;

    const int intervalMs = 1000 / SAMPLE_RATE_HZ; // 20ms

    for (int i = 0; i < TOTAL_SAMPLES; i++) {
        unsigned long start = millis();

        updateAngle();
        count++;

        // Welford update for pitch
        double deltaPitch = pitch - meanPitch;
        meanPitch += deltaPitch / count;
        M2Pitch += deltaPitch * (pitch - meanPitch);

        // Welford update for roll
        double deltaRoll = roll - meanRoll;
        meanRoll += deltaRoll / count;
        M2Roll += deltaRoll * (roll - meanRoll);

        // Progress every 5 seconds
        if (i % (SAMPLE_RATE_HZ * 5) == 0) {
            Serial.printf("[CAL] %d / %d seconds...\n",
                          i / SAMPLE_RATE_HZ, CALIBRATION_SECONDS);
        }

        // Pace the loop
        unsigned long elapsed = millis() - start;
        if (elapsed < intervalMs) delay(intervalMs - elapsed);
    }

    digitalWrite(LED_PIN, LOW);

    // Compute standard deviations
    float stdPitch = sqrtf((float) (M2Pitch / (count - 1)));
    float stdRoll = sqrtf((float) (M2Roll / (count - 1)));

    Serial.println("\n[CAL] Calibration complete.");
    Serial.printf("  Mean pitch : %.3f deg\n", (float) meanPitch);
    Serial.printf("  Mean roll  : %.3f deg\n", (float) meanRoll);
    Serial.printf("  Std  pitch : %.3f deg\n", stdPitch);
    Serial.printf("  Std  roll  : %.3f deg\n", stdRoll);

    // Store to NVS flash
    prefs.begin("dd_profile", false); // "dd" = drowsiness detector, rw mode
    prefs.putFloat("mean_pitch", (float) meanPitch);
    prefs.putFloat("mean_roll", (float) meanRoll);
    prefs.putFloat("std_pitch", stdPitch);
    prefs.putFloat("std_roll", stdRoll);
    prefs.putBool("calibrated", true);
    prefs.end();

    Serial.println("[CAL] Profile saved to flash.");
    Serial.println("      Flash Firmware 2 when ready to use the detector.");

    // Confirmation blink
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

void printStoredProfile() {
    prefs.begin("dd_profile", true); // read-only
    bool calibrated = prefs.getBool("calibrated", false);

    if (!calibrated) {
        Serial.println("[INFO] No stored profile found.");
        prefs.end();
        return;
    }

    Serial.println("[INFO] Stored profile found:");
    Serial.printf("  Mean pitch : %.3f deg\n", prefs.getFloat("mean_pitch", 0));
    Serial.printf("  Mean roll  : %.3f deg\n", prefs.getFloat("mean_roll", 0));
    Serial.printf("  Std  pitch : %.3f deg\n", prefs.getFloat("std_pitch", 0));
    Serial.printf("  Std  roll  : %.3f deg\n", prefs.getFloat("std_roll", 0));
    prefs.end();
}
