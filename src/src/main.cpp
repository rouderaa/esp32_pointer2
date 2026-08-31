#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <Arduino.h>
#include <Wire.h>    // only used briefly at boot, see ensureLidarUARTMode()
#include <TFMPlus.h> // Benewake TFMini-Plus/TFMini-S driver, UART mode
#include "BluetoothA2DPSource.h"

// ── Sensor mode auto-configuration ───────────────────────────────────────────
// The TFMini-S's two signal pins are dual-purpose: RX/TX in UART mode,
// SDA/SCL in I2C mode. If an earlier test left the sensor switched into I2C
// mode, ensureLidarUARTMode() (called once at boot, before the UART is
// opened) detects that over I2C and switches it back — no rewiring needed.

// ── Pin definitions ──────────────────────────────────────────────────────────
#define PIN_LED 22
#define PIN_KEY 23
#define PIN_LIDAR_RX 18 // ESP32 RX <- TFMini-S TX (== SDA in I2C mode)
#define PIN_LIDAR_TX 21 // ESP32 TX -> TFMini-S RX (== SCL in I2C mode)

// I2C address the sensor would be using if still in I2C mode from an
// earlier test — only used by ensureLidarUARTMode() below.
#define LIDAR_I2C_ADDR 0x10

// ── Distance / beep configuration ────────────────────────────────────────────
// The TFMini-S reports in cm; we convert to mm and work in mm throughout.
// 12 m is the sensor's full rated range.
const uint16_t DIST_MIN_MM = 100;   //  10 cm — closest expected
const uint16_t DIST_MAX_MM = 12000; // 1200 cm — furthest expected (sensor's full range)

// Inter-beep delay: short distance → short delay (rapid beeps)
const uint32_t BEEP_DELAY_MIN_MS = 80;  // ms at closest distance
const uint32_t BEEP_DELAY_MAX_MS = 800; // ms at furthest distance

// How long each beep toneState lasts (fixed)
const uint32_t BEEP_ON_MS = 60;

// ── Pitch configuration ───────────────────────────────────────────────────────
// The beep's pitch steps up a full octave for every 4 m *closer* the object
// is, on top of the continuous distance→rate mapping above. The farthest
// band (8-12 m) plays at BASE_FREQUENCY_HZ, each band closer than that
// doubles the frequency, so the nearest band (0-4 m) plays loudest/highest
// at 4x BASE_FREQUENCY_HZ. Swap back to an ascending mapping (drop the
// inversion in updateBeepSequencer()) if you'd rather have farther objects
// sound higher again.
const uint32_t OCTAVE_BAND_MM = 4000;   // every 4 m = one octave step
const float BASE_FREQUENCY_HZ = 220.0f; // pitch for the farthest band
const uint8_t OCTAVE_BAND_COUNT = (DIST_MAX_MM + OCTAVE_BAND_MM - 1) / OCTAVE_BAND_MM; // ceil

// ── Debug printing toggle ─────────────────────────────────────────────────────
// Serial.printf() at 115200 baud is not free — each call costs roughly 1-2 ms.
// That's negligible on its own, but turn this off once you're done tuning so it
// can't add jitter to the loop timing.
#define DEBUG_SENSOR 0

// Set to 1 to skip ensureLidarUARTMode() entirely — useful for isolating
// whether that boot-time I2C probe/switch step is itself part of a problem,
// now that the physical wiring has been confirmed good with a multimeter.
#define SKIP_I2C_MODE_CHECK 0

// ── Volume control configuration ─────────────────────────────────────────────
// Read from Serial: '+' increases volume, '-' decreases volume.
// BluetoothA2DPSource::set_volume() takes a uint8_t in the range 0-127.
const uint8_t VOLUME_MIN = 0;
const uint8_t VOLUME_MAX = 127;
const uint8_t VOLUME_STEP = 5;

// ── Mute button configuration ────────────────────────────────────────────────
// PIN_KEY is wired active-low (INPUT_PULLUP: idle HIGH, pressed reads LOW).
// A single press toggles mute; the LED reflects the current mute state
// rather than the raw, momentary button state.
const uint32_t KEY_DEBOUNCE_MS = 50;

// ── Global objects ────────────────────────────────────────────────────────────
BluetoothA2DPSource a2dp_source;
HardwareSerial lidarSerial(1); // ESP32 UART1, dedicated to the TFMini-S
TFMPlus tfmP;

// ── Global toneState state (written from loop, read from audio ISR-like callback) ──
struct ToneState
{
  volatile bool active;
  volatile float amplitude;
  volatile float frequency;
};

static ToneState toneState = {
    .active = false,
    .amplitude = 8000.0,
    .frequency = BASE_FREQUENCY_HZ};

// Mute flag — read by the audio callback (get_sound_data), written by
// updateMuteButton() in loop(). volatile for the same reason as toneState:
// it's shared across the loop() context and the Bluetooth audio task.
static volatile bool muted = false;

// ── Beep sequencer state ──────────────────────────────────────────────────────
// phaseStartMs marks when the current ON or OFF phase began. Unlike a fixed
// "next event" timestamp, we recompute the phase's *duration* from the latest
// beepIntervalMs on every single call — so if the measured distance suddenly
// changes mid-phase, the very next loop() iteration (≈10 ms later) reacts to
// it instead of waiting for the old, stale phase to finish.
static uint32_t phaseStartMs = 0;
static bool beepCurrentlyOn = false;
static uint32_t beepIntervalMs = BEEP_DELAY_MAX_MS;

// ── Distance-reading cache ────────────────────────────────────────────────────
// readDistanceMm() only gets a *new* sample when a complete frame has arrived
// over UART, but loop() runs far more often. We need to tell the difference
// between "no new frame yet" (keep using last good value) and "sensor
// reported an invalid/out-of-range reading" (actually go silent).
static uint16_t lastValidDistMm = 0;
static bool haveValidReading = false;

// ── Volume state ──────────────────────────────────────────────────────────────
static uint8_t currentVolume = 40; // matches the initial a2dp_source.set_volume(40) below

// ── float map helper (Arduino map() is integer-only) ─────────────────────────
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ─────────────────────────────────────────────────────────────────────────────
// TFMini-S I2C→UART mode switch — call once, before opening the UART.
//
// If a previous test left the sensor in I2C mode, it will answer at
// LIDAR_I2C_ADDR on the I2C bus. If it does, we send it the raw command
// bytes (from Benewake's TFMini-S protocol) to switch back to UART and save
// that setting:
//   - Set communication mode to UART: 5A 05 0A 00 69
//   - Save settings (required on the TFMini-S for the mode change to stick):
//     5A 04 11 6F
// If nothing answers on I2C, we assume the sensor is already in UART mode
// (its native, default state) and leave it alone. Either way, the I2C bus
// is released afterwards so the same pins are free to be reopened as UART.
//
// Returns true if an I2C-mode sensor was found and switched.
// ─────────────────────────────────────────────────────────────────────────────
bool ensureLidarUARTMode()
{
  Wire.begin(PIN_LIDAR_RX, PIN_LIDAR_TX);
  Wire.setClock(100000);
  delay(50);

  // Send the switch-to-UART + save-settings commands over I2C
  // unconditionally, rather than gating on an address probe first. A
  // marginal/under-pulled-up I2C bus can make that probe unreliable (it can
  // report "not found" even when the sensor is there), which would leave a
  // still-I2C-mode sensor stuck that way. If the sensor is actually already
  // in UART mode, these bytes are just meaningless noise on lines it's no
  // longer treating as I2C — harmless either way.
  Serial.println("TFMini-S: ensuring UART mode (sending switch-back over I2C)...");

  const uint8_t setSerialModeCmd[] = {0x5A, 0x05, 0x0A, 0x00, 0x69};
  Wire.beginTransmission(LIDAR_I2C_ADDR);
  Wire.write(setSerialModeCmd, sizeof(setSerialModeCmd));
  Wire.endTransmission();
  delay(50);

  const uint8_t saveSettingsCmd[] = {0x5A, 0x04, 0x11, 0x6F};
  Wire.beginTransmission(LIDAR_I2C_ADDR);
  Wire.write(saveSettingsCmd, sizeof(saveSettingsCmd));
  Wire.endTransmission();
  delay(300); // give the sensor time to apply the change

  Wire.end();

  // Explicitly release these pins from the I2C peripheral's grip before the
  // UART opens them — on some ESP32 core versions Wire.end() doesn't fully
  // detach the GPIO-matrix routing, which can leave I2C fighting the UART
  // peripheral for control of the same two wires right after.
  pinMode(PIN_LIDAR_RX, INPUT);
  pinMode(PIN_LIDAR_TX, INPUT);
  delay(50);

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// A2DP connection-state logging — lets us see on Serial whether the
// Bluetooth link to the speaker is actually being made at all, since no
// audio ever plays if it isn't (silently, with no other symptom).
// ─────────────────────────────────────────────────────────────────────────────
void a2dpConnectionStateChanged(esp_a2d_connection_state_t state, void *)
{
  const char *stateNames[] = {"DISCONNECTED", "CONNECTING", "CONNECTED", "DISCONNECTING"};
  Serial.printf("A2DP connection state: %s\n", stateNames[state]);
}

// ─────────────────────────────────────────────────────────────────────────────
// Audio callback — runs on the Bluetooth task; reads volatile toneState state only.
// ─────────────────────────────────────────────────────────────────────────────
int32_t get_sound_data(Frame *data, int32_t frame_count)
{
  static float phase = 0.0f;
  static float envelope = 0.0f; // 0..1, ramps toward toneState.active
  const float sampleRate = 44100.0f;

  // Time (in samples) for the envelope to ramp fully between silent and
  // full volume. An instant on/off jump is a step discontinuity, which a
  // speaker reproduces as an audible click/crack. A few milliseconds is
  // short enough to stay snappy against the ~60 ms beep length, but long
  // enough that the transition is inaudible as a click.
  const float ENVELOPE_RAMP_MS = 3.0f;
  const float envelopeStep = 1.0f / (ENVELOPE_RAMP_MS * 0.001f * sampleRate);

  for (int i = 0; i < frame_count; i++)
  {
    // Muting is folded in here (rather than in the sequencer) so the same
    // envelope ramp that avoids clicks on normal beep on/off also covers
    // mute on/off — no separate fade logic needed, and a mute toggled
    // mid-beep still ends cleanly instead of cutting off abruptly.
    float target = (toneState.active && !muted) ? 1.0f : 0.0f;
    if (envelope < target)
    {
      envelope += envelopeStep;
      if (envelope > target)
        envelope = target;
    }
    else if (envelope > target)
    {
      envelope -= envelopeStep;
      if (envelope < target)
        envelope = target;
    }

    int16_t sample = 0;
    if (envelope > 0.0f)
    {
      float a = toneState.amplitude * envelope;
      float f = toneState.frequency;
      sample = (int16_t)(a * 0.5161f * sinf(phase) +
                         a * 0.2581f * sinf(2.0f * phase) +
                         a * 0.1290f * sinf(4.0f * phase) +
                         a * 0.0645f * sinf(6.0f * phase) +
                         a * 0.0323f * sinf(8.0f * phase));
    }
    data[i].channel1 = sample;
    data[i].channel2 = sample;

    // Phase advances continuously (even while silent) so there's no phase
    // discontinuity if a new beep starts before the envelope has fully
    // settled — only the envelope gates the audible amplitude.
    phase += 2.0f * PI * toneState.frequency / sampleRate;
    if (phase > 2.0f * PI)
      phase -= 2.0f * PI;
  }
  return frame_count;
}

// ─────────────────────────────────────────────────────────────────────────────
// Mute button — call every loop iteration.
// Debounces PIN_KEY (active-low) and toggles `muted` on each press
// (HIGH -> LOW transition), so one press mutes and the next press unmutes.
// ─────────────────────────────────────────────────────────────────────────────
void updateMuteButton()
{
  static bool lastRawKeyState = HIGH;
  static bool debouncedKeyState = HIGH;
  static uint32_t lastKeyChangeMs = 0;

  uint32_t now = millis();
  bool rawKeyState = digitalRead(PIN_KEY);

  if (rawKeyState != lastRawKeyState)
  {
    lastKeyChangeMs = now;
    lastRawKeyState = rawKeyState;
  }

  if ((now - lastKeyChangeMs) >= KEY_DEBOUNCE_MS && rawKeyState != debouncedKeyState)
  {
    debouncedKeyState = rawKeyState;

    // Toggle only on the press edge (idle HIGH -> pressed LOW), not on release,
    // so holding the key down doesn't re-toggle anything.
    if (debouncedKeyState == LOW)
    {
      muted = !muted;
      Serial.printf("Mute: %s\n", muted ? "ON" : "OFF");
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// LED — reflects current mute state (on = muted), independent of whether
// the key is currently being held.
// ─────────────────────────────────────────────────────────────────────────────
void updateMuteLed()
{
  digitalWrite(PIN_LED, muted ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// Volume control — call every loop iteration.
// Reads '+'/'-' characters from Serial and adjusts a2dp_source volume,
// clamped to [VOLUME_MIN, VOLUME_MAX]. Prints the new volume on change.
// ─────────────────────────────────────────────────────────────────────────────
void updateVolumeControl()
{
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    bool changed = false;

    if (c == '+')
    {
      uint8_t newVolume = (currentVolume > VOLUME_MAX - VOLUME_STEP)
                              ? VOLUME_MAX
                              : currentVolume + VOLUME_STEP;
      changed = (newVolume != currentVolume);
      currentVolume = newVolume;
    }
    else if (c == '-')
    {
      uint8_t newVolume = (currentVolume < VOLUME_MIN + VOLUME_STEP)
                              ? VOLUME_MIN
                              : currentVolume - VOLUME_STEP;
      changed = (newVolume != currentVolume);
      currentVolume = newVolume;
    }
    else
    {
      // Ignore any other character (newlines, other input, etc.)
      continue;
    }

    if (changed)
    {
      a2dp_source.set_volume(currentVolume);
      // Serial.printf("Volume: %u\n", currentVolume);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// TFMini-S — read latest ranging result (mm).
//
// Returns:
//   - the last known-good distance if no complete frame is ready yet
//     (tfmP.getData() returns false) — this is NOT the same as "out of range"
//   - 0 if the sensor explicitly reports an invalid/unreliable reading
//   - the freshly read distance otherwise
// ─────────────────────────────────────────────────────────────────────────────
uint16_t readDistanceMm()
{
  int16_t tfDist = 0; // cm
  int16_t tfFlux = 0; // signal strength
  int16_t tfTemp = 0; // chip temperature

  if (!tfmP.getData(tfDist, tfFlux, tfTemp))
  {
    // No new complete frame parsed this call — reuse the last valid reading
    // so the beep sequencer keeps running smoothly between sensor updates.
    return haveValidReading ? lastValidDistMm : 0;
  }

#if DEBUG_SENSOR
  Serial.printf("Range: %d cm | Flux: %d | Temp: %d | Volume: %u\n",
                tfDist, tfFlux, tfTemp, currentVolume);
#endif

  // TFMini-S reports a non-positive distance, or a low signal-strength
  // (flux) value, when the reading is unreliable — target too close, too
  // far, low-reflectivity, or ambient light saturation.
  if (tfDist <= 0 || tfFlux < 100)
  {
    haveValidReading = false;
    return 0;
  }

  haveValidReading = true;
  lastValidDistMm = (uint16_t)(tfDist * 10); // cm -> mm
  return lastValidDistMm;
}

// ─────────────────────────────────────────────────────────────────────────────
// Beep sequencer — call every loop iteration.
// Updates toneState.active/frequency based on a non-blocking ON/OFF schedule
// derived from the measured distance: shorter distance → shorter inter-beep
// interval, and every 4 m of distance steps the pitch up one octave.
//
// The OFF-phase gap is recomputed from the *current* beepIntervalMs on every
// call (relative to when the phase started), instead of being locked in once
// when the phase began. That means a sudden distance change takes effect
// within one loop() iteration (~10 ms) instead of waiting for the previously
// scheduled, now-stale gap to finish.
//
// Note: this keeps scheduling beeps on its normal timing even while muted —
// muting is applied downstream, in the audio callback — so the moment mute
// is turned off, beeping resumes in sync with distance rather than needing
// to "catch up".
// ─────────────────────────────────────────────────────────────────────────────
void updateBeepSequencer(uint16_t distMm)
{
  uint32_t now = millis();

  // ── Update the interval and pitch from the latest distance reading ────────
  if (distMm >= DIST_MIN_MM && distMm <= DIST_MAX_MM)
  {
    beepIntervalMs = (uint32_t)mapf(
        (float)distMm,
        (float)DIST_MIN_MM, (float)DIST_MAX_MM,
        (float)BEEP_DELAY_MIN_MS, (float)BEEP_DELAY_MAX_MS);

    uint32_t band = distMm / OCTAVE_BAND_MM;
    if (band >= OCTAVE_BAND_COUNT)
      band = OCTAVE_BAND_COUNT - 1;
    uint32_t invertedBand = (OCTAVE_BAND_COUNT - 1) - band; // close = high pitch, far = low pitch
    toneState.frequency = BASE_FREQUENCY_HZ * (float)(1UL << invertedBand); // x2 per 4 m band
  }
  else
  {
    // Genuinely out of range (or no valid reading has ever been seen): silence.
    // distMm == 0 only reaches here when readDistanceMm() explicitly reported
    // an invalid status, not merely because a new sample wasn't ready yet.
    if (toneState.active || beepCurrentlyOn)
    {
      toneState.active = false;
      beepCurrentlyOn = false;
      phaseStartMs = now;
    }
    return;
  }

  // ── Non-blocking sequencer, re-evaluated live against beepIntervalMs ──────
  uint32_t elapsed = now - phaseStartMs;

  if (!beepCurrentlyOn)
  {
    // Waiting out the silent gap. Gap length is derived fresh each call, so
    // it shrinks/grows immediately as the distance (and thus interval) changes.
    uint32_t gapMs = (beepIntervalMs > BEEP_ON_MS)
                         ? (beepIntervalMs - BEEP_ON_MS)
                         : 0;
    if (elapsed >= gapMs)
    {
      toneState.active = true;
      beepCurrentlyOn = true;
      phaseStartMs = now;
    }
  }
  else
  {
    // Beep is sounding — fixed short ON duration, no need to react here.
    if (elapsed >= BEEP_ON_MS)
    {
      toneState.active = false;
      beepCurrentlyOn = false;
      phaseStartMs = now;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);

  // ── GPIO ──────────────────────────────────────────────────────────────────
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_KEY, INPUT_PULLUP);

  // ── TFMini-S startup sequence (UART) ────────────────────────────────────
  // Make sure the sensor isn't still left in I2C mode from an earlier test.
#if !SKIP_I2C_MODE_CHECK
  ensureLidarUARTMode();
#endif

  lidarSerial.begin(115200, SERIAL_8N1, PIN_LIDAR_RX, PIN_LIDAR_TX);
  delay(100);

#if DEBUG_SENSOR
  // Raw diagnostic: dump whatever shows up on the lidar UART for a second,
  // before attempting any command handshake with it. This tells us at a
  // glance whether the sensor is talking at all:
  //  - nothing printed        -> no data arriving (power, baud, or a wiring
  //                               issue the continuity check wouldn't catch,
  //                               e.g. swapped RX/TX)
  //  - random-looking bytes   -> something's on the line, but not framed
  //                               correctly (baud mismatch, TX/RX swapped)
  //  - repeating "59 59 ..."  -> the sensor is alive and streaming
  //                               correctly; a later failure is in the
  //                               command handshake, not basic connectivity
  Serial.println("TFMini-S: raw UART dump (1s)...");
  uint32_t dumpDeadline = millis() + 1000;
  uint16_t byteCount = 0;
  while (millis() < dumpDeadline)
  {
    if (lidarSerial.available())
    {
      Serial.printf("%02X ", lidarSerial.read());
      byteCount++;
    }
  }
  Serial.printf("\nTFMini-S: raw dump complete, %u bytes seen.\n", byteCount);
#endif

  tfmP.begin(&lidarSerial);

  // Verify the device is actually talking to us before continuing.
  if (tfmP.sendCommand(GET_FIRMWARE_VERSION, 0))
  {
    Serial.printf("TFMini-S firmware: V%d.%d.%d\n",
                  tfmP.version[0], tfmP.version[1], tfmP.version[2]);
  }
  else
  {
    Serial.println("TFMini-S init failed — check wiring!");
    while (true)
    {
      delay(100);
    }
  }

  // Put the sensor into continuous (streaming) output at a fixed frame rate.
  if (!tfmP.sendCommand(SET_FRAME_RATE, FRAME_100))
  {
    Serial.println("TFMini-S: failed to set frame rate.");
  }

  Serial.println("TFMini-S (UART) ready.");

  // ── Bluetooth A2DP ────────────────────────────────────────────────────────
  a2dp_source.set_auto_reconnect(true);
  a2dp_source.set_on_connection_state_changed(a2dpConnectionStateChanged);
  a2dp_source.start("B66", get_sound_data);

  a2dp_source.set_volume(currentVolume);
  Serial.printf("Volume: %u\n", currentVolume);

  phaseStartMs = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
  // ── Key → mute toggle, LED reflects mute state ───────────────────────────
  updateMuteButton();
  updateMuteLed();

  // ── Volume control via Serial ('+' / '-') ───────────────────────────────────
  updateVolumeControl();

  // ── Read TFMini-S ─────────────────────────────────────────────────────────
  uint16_t distMm = readDistanceMm();

  // if (distMm > 0) {
  //   Serial.printf("Distance: %u mm\n", distMm);
  // }

  // ── Drive beep sequencer from measured distance ────────────────────────────
  updateBeepSequencer(distMm);

#if DEBUG_SENSOR
  // Throttled heartbeat: confirms whether the beep sequencer is actually
  // going active, independent of whatever the TFMini-S debug line above
  // says — if this never shows active=1, no tone is ever being sent to the
  // A2DP callback regardless of whether Bluetooth is connected.
  static uint32_t lastHeartbeatMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastHeartbeatMs >= 1000)
  {
    lastHeartbeatMs = nowMs;
    Serial.printf("Heartbeat: dist=%u mm | active=%d | freq=%.0f Hz | a2dp_connected=%d | muted=%d\n",
                  distMm, toneState.active, toneState.frequency, a2dp_source.is_connected(), muted);
  }
#endif

  // Short sleep — keeps the loop responsive without busy-spinning.
  // The beep sequencer is non-blocking so this does not affect timing accuracy.
  delay(10);
}