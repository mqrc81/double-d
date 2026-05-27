/*
 * Drowsiness Data Collection Firmware
 * =====================================
 * Standalone hat ESP32. No ESP-NOW, no car reference signal.
 * Records simulated drowsy head-drop episodes to NVS for later
 * threshold derivation.
 *
 * Hardware
 * --------
 *   MPU-6050  : SDA=21, SCL=22
 *   LED       : GPIO4 (330Ω to GND)
 *   Button A  : GPIO0 (built-in Boot button) — marks ONSET of head drop
 *   Button B  : GPIO2 — marks PEAK (lowest point) of head drop
 *
 * Button wiring (Button B)
 * ------------------------
 *   GPIO2 ── one leg of button
 *   GND   ── other leg of button
 *   No external resistor needed (INPUT_PULLUP used).
 *
 * Session flow
 * ------------
 *   1. Power on. LED blinks 1Hz.
 *   2. Sit upright, head still. Press A → 10s baseline collection begins
 *      (fast blink). Stores resting mean_pitch and std_pitch.
 *   3. LED goes solid. Ready for episodes.
 *   4. Let head start dropping → press A at onset.
 *      LED off during drop.
 *   5. At lowest point → press B (peak).
 *      1 short beep. Head snaps back naturally.
 *   6. Repeat from step 4 for as many episodes as desired (target: 10+).
 *   7. Press A again after all episodes → finalises, saves summary to NVS,
 *      3 LED flashes.
 *
 * NVS namespace: "dd_data"
 * Keys stored
 * -----------
 *   baseline_mean   float   Mean pitch while sitting alert and upright
 *   baseline_std    float   Std of pitch during baseline (natural sway)
 *   episode_count   int     Total episodes recorded this session
 *   ep_N_onset      float   Pitch at onset button press (episode N)
 *   ep_N_peak       float   Pitch at peak button press
 *   ep_N_delta      float   peak - onset (signed, positive = forward drop)
 *   ep_N_duration   int     ms between onset and peak press
 *   ep_N_drop_rate  float   deg/s over onset→peak window (slope)
 *   ep_N_onset_rate float   deg/s in 500ms window just before onset press
 *                           (pre-drop drift — useful for early detection)
 *   summary_mean_delta      float   Mean of all ep_N_delta
 *   summary_std_delta       float   Std of all ep_N_delta
 *   summary_mean_drop_rate  float   Mean of all ep_N_drop_rate
 *   summary_std_drop_rate   float
 *   summary_mean_duration   float   Mean episode duration ms
 *   summary_std_duration    float
 *
 * Why these features
 * ------------------
 *   delta (peak - onset): Relative drop magnitude. Using absolute pitch
 *   would break across sessions due to hat placement variance. Delta from
 *   the same-session onset is placement-invariant.
 *
 *   drop_rate: How fast the head falls. Drowsy nods are typically slower
 *   than voluntary bows. This is the most discriminative single feature.
 *
 *   onset_rate: Pre-drop drift rate. In real drowsiness, the head often
 *   drifts slightly before the actual nod begins. Capturing this enables
 *   earlier detection in the DD firmware.
 *
 *   duration: Drowsy episodes tend to be sustained (2-5s). Voluntary
 *   movements are faster. Used to gate false positives.
 *
 *   baseline_std: Encodes your personal resting movement variance.
 *   The DD firmware uses this to set stillness thresholds and normalise
 *   the other features across individuals.
 *
 * DD firmware threshold derivation (for reference)
 * -------------------------------------------------
 *   angle_threshold  = baseline_mean + summary_mean_delta * 0.7
 *   rate_threshold   = summary_mean_drop_rate * 0.6   // trigger before peak
 *   stillness_thresh = baseline_std * 0.3             // unusually still = pre-drowsy
 *   min_duration_ms  = summary_mean_duration * 0.5    // must be sustained
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <math.h>

// ─── Pin definitions ──────────────────────────────────────────────
#define SDA_PIN         21
#define SCL_PIN         22
#define LED_PIN         4
#define BUTTON_A_PIN    13       // Boot button — onset marker
#define BUTTON_B_PIN    14       // Peak marker

// ─── Sampling ─────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ          20
#define SAMPLE_INTERVAL_US      (1000000 / SAMPLE_RATE_HZ)

// ─── Complementary filter ─────────────────────────────────────────
#define ALPHA                   0.96f

// ─── Baseline collection ──────────────────────────────────────────
#define BASELINE_DURATION_MS    10000   // 10 seconds

// ─── Pre-drop drift window ────────────────────────────────────────
// How many samples to look back when computing onset_rate.
// 10 samples at 20Hz = 500ms — captures slow pre-nod drift.
#define ONSET_RATE_WINDOW       10

// ─── Max episodes per session ─────────────────────────────────────
#define MAX_EPISODES            32

// ─── State machine ────────────────────────────────────────────────
typedef enum {
    STATE_IDLE, // Waiting for baseline to begin
    STATE_BASELINE, // Collecting resting pitch stats
    STATE_READY, // Baseline done, ready for episodes
    STATE_IN_EPISODE, // Onset pressed, waiting for peak
    STATE_SAVING, // All episodes done, saving summary
    STATE_DONE // Complete
} CollectState;

CollectState state = STATE_IDLE;

// ─── IMU ──────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
float pitch = 0.0f;
int64_t lastTimerUs = 0;

// ─── Pitch history ring buffer ────────────────────────────────────
// Stores the last ONSET_RATE_WINDOW samples for onset_rate computation.
float pitchHistory[ONSET_RATE_WINDOW];
int historyIdx = 0;
bool historyFull = false;

// ─── Baseline (Welford) ───────────────────────────────────────────
double bMean = 0.0, bM2 = 0.0;
int bCount = 0;
unsigned long baselineStartMs = 0;
float baselineMean = 0.0f;
float baselineStd = 0.0f;

// ─── Episode state ────────────────────────────────────────────────
struct Episode {
    float onset_pitch; // pitch at onset press
    float peak_pitch; // pitch at peak press
    float delta; // peak - onset
    float drop_rate; // deg/s from onset to peak
    float onset_rate; // deg/s in 500ms before onset
    int duration_ms; // onset→peak ms
    unsigned long onset_time_ms;
};

Episode episodes[MAX_EPISODES];
int episodeCount = 0;
bool inEpisode = false;

// For computing drop_rate: pitch value snapshotted at onset press
float onsetPitch = 0.0f;
unsigned long onsetMs = 0;

// ─── NVS ──────────────────────────────────────────────────────────
Preferences prefs;

// ─── Forward declarations ─────────────────────────────────────────
void updateIMU();

void updateHistory(float p);

float computeRateOverHistory(); // slope over stored window
float computeRateOverInterval(float startPitch, float endPitch, int durationMs);

void handleButtons();

void onButtonA();

void onButtonB();

void runBaseline();

void finaliseSession();

void saveEpisode(int idx);

void saveSummary(float a, float b, float c, float d, float e, float f);

void flashLED(int count, int onMs = 120, int offMs = 120);

void setLED(bool on);

// ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_A_PIN, INPUT_PULLUP);
    pinMode(BUTTON_B_PIN, INPUT_PULLUP);
    setLED(true);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU-6050 not found. Check wiring.");
        // Rapid error blink — halts here
        while (true) {
            for (int i = 0; i < 5; i++) {
                setLED(true);
                delay(80);
                setLED(false);
                delay(80);
            }
            delay(600);
        }
    }
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[OK] MPU-6050 connected.");

    // Print any previously stored profile
    prefs.begin("dd_data", true);
    int prevCount = prefs.getInt("episode_count", 0);
    if (prevCount > 0) {
        Serial.printf("[NVS] Previous session: %d episodes stored.\n", prevCount);
        Serial.printf("      baseline_mean=%.3f  baseline_std=%.3f\n",
                      prefs.getFloat("baseline_mean", 0),
                      prefs.getFloat("baseline_std", 0));
        Serial.printf("      summary_mean_delta=%.3f  summary_mean_drop_rate=%.3f\n",
                      prefs.getFloat("summary_mean_delta", 0),
                      prefs.getFloat("summary_mean_drop_rate", 0));
    } else {
        Serial.println("[NVS] No previous session found.");
    }
    prefs.end();

    memset(pitchHistory, 0, sizeof(pitchHistory));
    lastTimerUs = esp_timer_get_time();

    Serial.println();
    Serial.println("[READY] Session flow:");
    Serial.println("  1. Sit upright, head still.");
    Serial.println("  2. Press A (Boot) → 10s baseline collection.");
    Serial.println("  3. LED solid → ready for episodes.");
    Serial.println("  4. Let head drop → press B at onset.");
    Serial.println("  5. At lowest point → press B (peak). 1 flash.");
    Serial.println("  6. Repeat step 4-5 for all episodes.");
    Serial.println("  7. Press A again → save and finish.");
    Serial.println();
}

// ─────────────────────────────────────────────────────────────────
void loop() {
    static int64_t lastSampleUs = 0;
    static unsigned long lastBlinkMs = 0;
    static bool blinkState = false;

    int64_t nowUs = esp_timer_get_time();
    if (nowUs - lastSampleUs < SAMPLE_INTERVAL_US) return;
    lastSampleUs = nowUs;

    updateIMU();
    updateHistory(pitch);
    handleButtons();

    unsigned long nowMs = millis();

    switch (state) {
        case STATE_IDLE:
            // 1Hz blink: waiting for baseline trigger
            if (nowMs - lastBlinkMs > 1000) {
                blinkState = !blinkState;
                setLED(blinkState);
                lastBlinkMs = nowMs;
            }
            break;

        case STATE_BASELINE:
            runBaseline();
            // 5Hz blink during collection
            if (nowMs - lastBlinkMs > 200) {
                blinkState = !blinkState;
                setLED(blinkState);
                lastBlinkMs = nowMs;
            }
            break;

        case STATE_READY:
            setLED(true); // Solid on = ready
            break;

        case STATE_IN_EPISODE:
            setLED(false); // LED off during drop — clear visual feedback
            // Print live pitch so you can verify the sensor is tracking
            static unsigned long lastLivePrint = 0;
            if (nowMs - lastLivePrint > 200) {
                lastLivePrint = nowMs;
                // Serial.printf("[LIVE] pitch=%.2f  delta=%.2f  ep=%d\n",
                // pitch,
                // pitch - onsetPitch,
                // episodeCount + 1);
            }
            break;

        case STATE_SAVING:
            finaliseSession();
            state = STATE_DONE;
            break;

        case STATE_DONE:
            // 3 slow flashes then steady off
            setLED(false);
            break;
    }
}

// ─────────────────────────────────────────────────────────────────
void updateIMU() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int64_t nowUs = esp_timer_get_time();
    float dt = (nowUs - lastTimerUs) / 1e6f;
    lastTimerUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) dt = SAMPLE_INTERVAL_US / 1e6f;

    float accelPitch = atan2f(a.acceleration.y, a.acceleration.z) * 180.0f / PI;
    float gyroPitchRate = g.gyro.x * 180.0f / PI;

    pitch = ALPHA * (pitch + gyroPitchRate * dt) + (1.0f - ALPHA) * accelPitch;
}

// ─────────────────────────────────────────────────────────────────
void updateHistory(float p) {
    pitchHistory[historyIdx] = p;
    historyIdx = (historyIdx + 1) % ONSET_RATE_WINDOW;
    if (historyIdx == 0) historyFull = true;
}

// Returns deg/s slope over the stored ring buffer window.
// Positive = head moving forward/downward.
float computeRateOverHistory() {
    int n = historyFull ? ONSET_RATE_WINDOW : historyIdx;
    if (n < 2) return 0.0f;

    int firstIdx = historyFull ? historyIdx : 0;
    float first = pitchHistory[firstIdx];
    float last = pitchHistory[(historyIdx - 1 + ONSET_RATE_WINDOW) % ONSET_RATE_WINDOW];
    float window = (float) (n - 1) / (float) SAMPLE_RATE_HZ;
    return (last - first) / window;
}

float computeRateOverInterval(float startPitch, float endPitch, int durationMs) {
    if (durationMs <= 0) return 0.0f;
    return (endPitch - startPitch) / (durationMs / 1000.0f);
}

// ─────────────────────────────────────────────────────────────────
// Baseline: Welford online mean/std of pitch while sitting still.
void runBaseline() {
    bCount++;
    double delta = pitch - bMean;
    bMean += delta / bCount;
    bM2 += delta * (pitch - bMean);

    // Progress every 2 seconds
    static unsigned long lastLogMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastLogMs >= 2000) {
        lastLogMs = nowMs;
        unsigned long elapsed = nowMs - baselineStartMs;
        Serial.printf("[BASE] %.1fs / %.1fs  pitch=%.2f  running_mean=%.3f\n",
                      elapsed / 1000.0f,
                      BASELINE_DURATION_MS / 1000.0f,
                      pitch,
                      (float) bMean);
    }

    if (millis() - baselineStartMs >= BASELINE_DURATION_MS) {
        baselineMean = (float) bMean;
        baselineStd = (bCount > 1) ? sqrtf((float) (bM2 / (bCount - 1))) : 0.0f;

        Serial.println("[BASE] Baseline complete.");
        Serial.printf("  mean_pitch = %.3f deg\n", baselineMean);
        Serial.printf("  std_pitch  = %.3f deg\n", baselineStd);
        Serial.println("  LED solid = ready. Press A at onset, B at peak.");

        // Save baseline immediately in case of power loss
        // prefs.begin("dd_data", false);
        // prefs.putFloat("baseline_mean", baselineMean);
        // prefs.putFloat("baseline_std", baselineStd);
        // prefs.end();

        flashLED(1);
        state = STATE_READY;
    }
}

// ─────────────────────────────────────────────────────────────────
// Button A: onset (in STATE_READY) or finalise (after episodes in STATE_READY)
// Button B: peak (only valid during STATE_IN_EPISODE)
void handleButtons() {
    // ── Button A ──
    {
        static bool lastRaw = LOW, wasPressed = false;
        static unsigned long lastDebounce = 0;
        bool raw = digitalRead(BUTTON_A_PIN);
        if (raw != lastRaw) {
            lastDebounce = millis();
            lastRaw = raw;
        }
        if (millis() - lastDebounce >= 50) {
            if (raw == HIGH && !wasPressed) {
                wasPressed = true;
                onButtonA();
            } else if (raw == LOW) {
                wasPressed = false;
            }
        }
    }

    // ── Button B ──
    {
        static bool lastRaw = LOW, wasPressed = false;
        static unsigned long lastDebounce = 0;
        bool raw = digitalRead(BUTTON_B_PIN);
        if (raw != lastRaw) {
            lastDebounce = millis();
            lastRaw = raw;
        }
        if (millis() - lastDebounce >= 50) {
            if (raw == HIGH && !wasPressed) {
                wasPressed = true;
                onButtonB();
            } else if (raw == LOW) {
                wasPressed = false;
            }
        }
    }
}

// Used to start and end the session
void onButtonA() {
    switch (state) {
        case STATE_IDLE:
            // Begin baseline
            Serial.println("[BASE] Starting 10s baseline. Sit still and upright.");
            bMean = 0.0;
            bM2 = 0.0;
            bCount = 0;
            baselineStartMs = millis();
            state = STATE_BASELINE;
            break;

        case STATE_BASELINE:
        case STATE_READY:
        case STATE_IN_EPISODE:
            Serial.printf("[DONE] Finalising %d episodes.\n", episodeCount);
            state = STATE_SAVING;

            break;
        default:
            break;
    }
}

// Used to mark start and end of drowsiness episode
void onButtonB() {
    switch (state) {
        case STATE_IDLE:
        case STATE_BASELINE:
        case STATE_SAVING:
            Serial.println("[WARN] Button B pressed outside of episode — ignored.");
            break;
        case STATE_READY:
            if (!inEpisode) {
                if (episodeCount == 0) {
                    // First episode onset
                    Serial.printf("[EP %d] Onset at pitch=%.2f  Let head drop, press B at lowest point.\n",
                                  episodeCount + 1, pitch);
                    onsetPitch = pitch;
                    onsetMs = millis();
                    // Capture pre-drop drift rate (what was happening before the press)
                    episodes[episodeCount].onset_rate = computeRateOverHistory();
                    episodes[episodeCount].onset_pitch = onsetPitch;
                    episodes[episodeCount].onset_time_ms = onsetMs;
                    inEpisode = true;
                    state = STATE_IN_EPISODE;
                } else {
                    // A press in STATE_READY after at least one episode = either
                    // new onset OR finalise request. Treat as onset if we have
                    // room, otherwise finalise.
                    if (episodeCount < MAX_EPISODES) {
                        Serial.printf("[EP %d] Onset at pitch=%.2f\n",
                                      episodeCount + 1, pitch);
                        onsetPitch = pitch;
                        onsetMs = millis();
                        episodes[episodeCount].onset_rate = computeRateOverHistory();
                        episodes[episodeCount].onset_pitch = onsetPitch;
                        episodes[episodeCount].onset_time_ms = onsetMs;
                        inEpisode = true;
                        state = STATE_IN_EPISODE;
                    }
                }
            }
            break;
        case STATE_IN_EPISODE:
            unsigned long peakMs = millis();
            int durationMs = (int) (peakMs - onsetMs);
            float peakPitch = pitch;

            Episode &ep = episodes[episodeCount];
            ep.peak_pitch = peakPitch;
            ep.delta = peakPitch - ep.onset_pitch;
            ep.duration_ms = durationMs;
            ep.drop_rate = computeRateOverInterval(ep.onset_pitch, peakPitch, durationMs);

            episodeCount++;

            Serial.printf("[EP %d] Peak recorded.\n", episodeCount);
            Serial.printf("  onset=%.2f deg  peak=%.2f deg\n", ep.onset_pitch, ep.peak_pitch);
            Serial.printf("  delta=%.2f deg  duration=%d ms\n", ep.delta, ep.duration_ms);
            Serial.printf("  drop_rate=%.2f deg/s  onset_rate=%.2f deg/s\n",
                          ep.drop_rate, ep.onset_rate);
            Serial.println("  Head back up. Press B for next onset.");

            // Save episode immediately to NVS (non-blocking — Preferences is fine at this rate)
            // saveEpisode(episodeCount - 1);

            flashLED(1, 80, 0);
            inEpisode = false;
            state = STATE_READY;
            break;
        default: break;
    }
}

// ─────────────────────────────────────────────────────────────────
void finaliseSession() {
    if (episodeCount == 0) {
        Serial.println("[DONE] No episodes to save.");
        flashLED(5, 60, 60);
        return;
    }

    Serial.println("[DONE] Session summary:");

    // Welford pass over episodes for summary stats
    double mDelta = 0, m2Delta = 0;
    double mRate = 0, m2Rate = 0;
    double mDur = 0, m2Dur = 0;
    for (int i = 0; i < episodeCount; i++) {
        int n = i + 1;
        double dDelta = episodes[i].delta - mDelta;
        double dRate = episodes[i].drop_rate - mRate;
        double dDur = episodes[i].duration_ms - mDur;
        mDelta += dDelta / n;
        m2Delta += dDelta * (episodes[i].delta - mDelta);
        mRate += dRate / n;
        m2Rate += dRate * (episodes[i].drop_rate - mRate);
        mDur += dDur / n;
        m2Dur += dDur * (episodes[i].duration_ms - mDur);
    }

    float stdDelta = (episodeCount > 1) ? sqrtf((float) (m2Delta / (episodeCount - 1))) : 0.0f;
    float stdRate = (episodeCount > 1) ? sqrtf((float) (m2Rate / (episodeCount - 1))) : 0.0f;
    float stdDur = (episodeCount > 1) ? sqrtf((float) (m2Dur / (episodeCount - 1))) : 0.0f;

    Serial.printf("  episodes            = %d\n", episodeCount);
    Serial.printf("  baseline_mean       = %.3f deg\n", baselineMean);
    Serial.printf("  baseline_std        = %.3f deg\n", baselineStd);
    Serial.printf("  mean_delta          = %.3f deg  (std=%.3f)\n", (float) mDelta, stdDelta);
    Serial.printf("  mean_drop_rate      = %.3f deg/s (std=%.3f)\n", (float) mRate, stdRate);
    Serial.printf("  mean_duration_ms    = %.0f ms   (std=%.0f)\n", (float) mDur, stdDur);
    Serial.println();
    Serial.println("[TIP] Suggested DD thresholds (adjust based on these numbers):");
    Serial.printf("  angle_threshold     = %.2f deg  (baseline_mean + mean_delta*0.7)\n",
                  baselineMean + (float) mDelta * 0.7f);
    Serial.printf("  rate_threshold      = %.2f deg/s (mean_drop_rate * 0.6)\n", (float) mRate * 0.6f);
    Serial.printf("  stillness_thresh    = %.3f deg  (baseline_std * 0.3)\n"baselineStd * 0.3f);
    Serial.printf("  min_duration_ms     = %.0f ms  (mean_duration_ms * 0.5)\n", (float) mDur * 0.5f);

    // saveSummary((float) mDelta, stdDelta, (float) mRate, stdRate, (float) mDur, stdDur);
    // Serial.println("[DONE] All data saved to NVS 'dd_data'.");
    // Serial.println("       Flash DD firmware to use these thresholds.");

    flashLED(3, 200, 150);
}

// ─────────────────────────────────────────────────────────────────
void saveEpisode(int idx) {
    char key[24];
    prefs.begin("dd_data", false);
    snprintf(key, sizeof(key), "ep_%d_onset", idx);
    prefs.putFloat(key, episodes[idx].onset_pitch);
    snprintf(key, sizeof(key), "ep_%d_peak", idx);
    prefs.putFloat(key, episodes[idx].peak_pitch);
    snprintf(key, sizeof(key), "ep_%d_delta", idx);
    prefs.putFloat(key, episodes[idx].delta);
    snprintf(key, sizeof(key), "ep_%d_duration", idx);
    prefs.putInt(key, episodes[idx].duration_ms);
    snprintf(key, sizeof(key), "ep_%d_droprate", idx);
    prefs.putFloat(key, episodes[idx].drop_rate);
    snprintf(key, sizeof(key), "ep_%d_onsetrate", idx);
    prefs.putFloat(key, episodes[idx].onset_rate);
    prefs.putInt("episode_count", episodeCount);
    prefs.end();
}

void saveSummary(float mDelta, float sDelta, float mRate, float sRate, float mDur, float sDur) {
    prefs.begin("dd_data", false);
    prefs.putFloat("summary_mean_delta", mDelta);
    prefs.putFloat("summary_std_delta", sDelta);
    prefs.putFloat("summary_mean_drop_rate", mRate);
    prefs.putFloat("summary_std_drop_rate", sRate);
    prefs.putFloat("summary_mean_duration", mDur);
    prefs.putFloat("summary_std_duration", sDur);
    prefs.end();
}

// ─────────────────────────────────────────────────────────────────
void flashLED(int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) {
        setLED(true);
        delay(onMs);
        setLED(false);
        if (i < count - 1) delay(offMs);
    }
}

void setLED(bool on) {
    digitalWrite(LED_PIN, on ? HIGH : LOW);
}
