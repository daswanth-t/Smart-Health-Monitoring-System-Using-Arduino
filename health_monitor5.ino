/*
  Smart Health Monitor - Arduino Uno  (version 5)

  Measures SpO2, heart rate (BPM) and body temperature.
  Shows values on a 16x2 I2C LCD. Beeps the buzzer and lights the LED
  when SpO2 < 90% (6 of the last 8 readings) or temperature > 38 C. The alert stops by itself when
  readings return to normal or the finger is removed.

  WIRING
  MAX30102 (HW-605): VIN->3.3V (Arduino 3.3V pin), GND->GND, SDA->A4, SCL->A5
  I2C LCD          : VCC->5V, GND->GND, SDA->A4, SCL->A5
  DS18B20          : red->5V, black->GND, yellow (DATA)->D2
                     4.7k resistor between DATA and 5V
  Buzzer           : (+) -> D8, (-) -> GND
  LED              : D9 -> 220 ohm -> LED long leg (+), short leg (-) -> GND

  LIBRARIES (Arduino IDE > Library Manager)
  - SparkFun MAX3010x Pulse and Proximity Sensor Library
  - LiquidCrystal I2C
  - OneWire
  - DallasTemperature

  MEMORY FIX (needed on the Uno, which has only 2048 bytes of RAM)
  Open  Documents/Arduino/libraries/SparkFun_MAX3010x_.../src/spo2_algorithm.h
  Find the line  #define BUFFER_SIZE (FS * 4)   (or similar) and change it to
      #define BUFFER_SIZE 75
  Save, restart the Arduino IDE, upload. RAM use should drop to about 77%.
  This sketch adapts automatically to whatever BUFFER_SIZE the library uses.

  HOW TO TUNE
  1. Keep DEBUG_IR = 1 and open the Serial Monitor at 115200 baud.
  2. Note the IR value with NO finger and with a finger on the sensor.
  3. Set FINGER_THRESHOLD between those two numbers.
  4. Leave SAMPLE_SHIFT at 2 (safe at any signal level).
  5. Heart rate appears about 7 seconds after the finger lands (it needs 5 steady beats).
     Rest your hand, press lightly and keep still. "---" means no steady rhythm yet.
  6. Still unstable? Set DEBUG_RAW to 1, rest a finger on the sensor for 30 seconds,
     and send the numbers from the Serial Monitor.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MAX30105.h>          // SparkFun library, also works for MAX30102
#include <spo2_algorithm.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---------- Settings you can change ----------
#define LCD_ADDR        0x27     // try 0x3F if the screen stays blank
#define ONE_WIRE_PIN    2
#define BUZZER_PIN      8
#define LED_PIN         9
#define DEBUG_IR        1        // 1 = print a summary line to the Serial Monitor every second
#define DEBUG_RAW       0        // 1 = print every raw IR sample instead (25 lines per second, for tuning)

const int      SPO2_LIMIT       = 90;      // alert below this (%)
const int      SPO2_MIN_VALID   = 70;      // values below this are treated as noise
const float    TEMP_LIMIT       = 38.0;    // alert above this (C)
const float    TEMP_OFFSET      = 0.8;     // skin-to-body offset; set after comparing with a thermometer
const uint32_t FINGER_THRESHOLD = 30000;   // raw IR below this = no finger (tune with DEBUG_IR)
const byte     LED_BRIGHTNESS   = 40;      // 0-255 (about 0.2 mA per step); lower if readings saturate
const byte     SAMPLE_SHIFT     = 2;       // scales samples for the SpO2 algorithm; 2 is safe at any signal level
const unsigned long BEEP_MS     = 500;     // beep on/off time

// ---------- Objects ----------
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
MAX30105 particleSensor;
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// ---------- Heart-rate beat detector ----------
// Replaces the library's heart-rate result, which was too noisy. Works on every IR sample:
// removes the slow baseline, smooths, then counts upward crossings of an adaptive threshold.
// A heart rate is shown only when the last beat intervals agree with each other.
// BEGIN BeatDetector
#define SAMPLE_MS      40     // 25 samples per second (200 Hz / 8 averaging)
#define WARMUP_SAMPLES 50     // ignore the first 2 seconds after the finger lands

class BeatDetector {
 public:
  int bpm;                    // 0 = no valid heart rate yet

  BeatDetector() { reset(); }

  void reset() {
    dc = 0; amp = 0; idx = 0;
    for (byte i = 0; i < 4; i++) ma[i] = 0;
    warm = 0; armed = false; haveBeat = false;
    sinceBeat = 0; nInt = 0; iPos = 0; bpm = 0;
  }

  void update(uint32_t ir) {
    if (ir < FINGER_THRESHOLD) { reset(); return; }
    float x = (float)ir;
    if (warm < WARMUP_SAMPLES) {          // follow the signal closely while the finger settles
      dc = x;
      for (byte i = 0; i < 4; i++) ma[i] = 0;
      warm++;
      return;
    }
    dc += (x - dc) * 0.12f;               // baseline filter: removes slow drift and breathing waves
    ma[idx] = x - dc;
    idx = (idx + 1) & 3;
    float s = (ma[0] + ma[1] + ma[2] + ma[3]) * 0.25f;   // 4-sample smoothing
    float u = -s;                               // reflected IR dips with each beat, so flip it
    amp = (u > amp) ? u : amp * 0.98f;          // slowly decaying peak size
    if (sinceBeat < 250) sinceBeat++;

    // No pulse (signal too small) or movement (too large): start over
    if (amp < 0.0008f * dc || amp > 0.05f * dc) { dropBeats(); return; }

    if (u < 0) armed = true;
    byte refr = 8;                        // never closer than 0.32 s
    if (nInt >= 3) {                      // ignore ripples inside 60% of the usual interval
      byte r = (byte)((0.6f * medianMs()) / SAMPLE_MS);
      if (r > refr) refr = r;
    }
    if (armed && u > 0.3f * amp && sinceBeat >= refr) {
      armed = false;
      if (haveBeat) addInterval((int)sinceBeat * SAMPLE_MS);
      haveBeat = true;
      sinceBeat = 0;
    }
  }

 private:
  float dc, amp, ma[4];
  byte idx, warm, sinceBeat, nInt, iPos;
  bool armed, haveBeat;
  int ints[5];

  void dropBeats() { haveBeat = false; armed = false; nInt = 0; iPos = 0; bpm = 0; }

  int medianMs() {
    int t[5];
    for (byte i = 0; i < nInt; i++) t[i] = ints[i];
    for (byte i = 1; i < nInt; i++) {     // insertion sort
      int v = t[i]; int j = i - 1;
      while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
      t[j + 1] = v;
    }
    return t[nInt / 2];
  }

  void addInterval(int ms) {
    if (nInt >= 3) {                      // one weak beat missed? then the gap is about twice normal
      int m = medianMs();
      if (ms > m * 17 / 10 && ms < m * 23 / 10) ms /= 2;
    }
    if (ms > 1500) { nInt = 0; iPos = 0; bpm = 0; return; }   // longer gap: start over
    if (ms < 320) return;
    ints[iPos] = ms;
    iPos = (iPos + 1) % 5;
    if (nInt < 5) nInt++;
    if (nInt < 4) { bpm = 0; return; }
    int med = medianMs();
    long sum = 0; byte good = 0;
    for (byte i = 0; i < nInt; i++) {
      int d = ints[i] - med; if (d < 0) d = -d;
      if (d <= med / 4) { sum += ints[i]; good++; }
    }
    bpm = (good >= 3) ? (int)(60000L * good / sum) : 0;
  }
};
// END BeatDetector

// ---------- Sample buffers (16-bit on Uno to save RAM) ----------
#ifndef BUFFER_SIZE
#define BUFFER_SIZE 100
#endif
#define NEW_SAMPLES 25           // samples collected per update (about 1 second)
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[BUFFER_SIZE];
uint16_t redBuffer[BUFFER_SIZE];
#else
uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
#endif

int32_t  spo2 = 0;
int8_t   validSPO2 = 0;
int32_t  heartRate = 0;
int8_t   validHeartRate = 0;
uint32_t lastIR = 0;          // full 32-bit IR value, used for finger detection
uint32_t lastRed = 0;         // last red value (debug print only)
bool     fingerOn = false;
bool     spo2Ok = false;      // SpO2 is valid and plausible
byte     lowHist = 0;         // last 8 SpO2 results, 1 = below the limit (newest in bit 0)
int      spo2Hist[5];         // last 5 valid SpO2 results; their median is shown
byte     spo2N = 0, spo2Pos = 0, spo2Bad = 0;
int      spo2Shown = 0;       // SpO2 shown on screen (0 = not available)

BeatDetector beat;
int hrShown = 0;             // heart rate shown on screen (0 = not available)
byte hrHold = 0;             // seconds the last heart rate is kept when the detector briefly loses it
byte hrJump = 0;             // seconds a sudden big change in heart rate has lasted
unsigned long lastLoopEnd = 0;
unsigned int spsX10 = 0;     // measured samples per second x 10 (debug)

// ---------- Temperature state ----------
float bodyTemp = NAN;
bool tempPending = false;
unsigned long tempRequestTime = 0;

// ---------- Alert state ----------
bool beepState = false;
unsigned long lastBeep = 0;

// Non-blocking temperature read (DS18B20 needs about 750 ms)
void updateTemperature() {
  if (!tempPending) {
    sensors.requestTemperatures();
    tempRequestTime = millis();
    tempPending = true;
  } else if (millis() - tempRequestTime >= 800) {
    float t = sensors.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) {
      bodyTemp = t + TEMP_OFFSET;
    }
    tempPending = false;
  }
}

bool alertActive() {
  bool lowO2 = fingerOn && __builtin_popcount(lowHist) >= 6;   // 6 of the last 8 results low
  bool fever = !isnan(bodyTemp) && bodyTemp > TEMP_LIMIT;
  return lowO2 || fever;
}

// Beep pattern: 0.5 s on, 0.5 s off while the alert is active; silent otherwise
void updateAlert() {
  if (alertActive()) {
    digitalWrite(LED_PIN, HIGH);
    if (millis() - lastBeep >= BEEP_MS) {
      beepState = !beepState;
      digitalWrite(BUZZER_PIN, beepState ? HIGH : LOW);
      lastBeep = millis();
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    beepState = false;
  }
}

// Print an integer right-aligned in 3 characters
void print3(int v) {
  if (v < 10) lcd.print(F("  "));
  else if (v < 100) lcd.print(' ');
  lcd.print(v);
}

void showReadings() {
  lcd.setCursor(0, 0);
  if (!fingerOn) {
    lcd.print(F("Place finger... "));
  } else {
    lcd.print(F("SpO2:"));
    if (spo2Shown > 0) print3(spo2Shown); else lcd.print(F("---"));
    lcd.print(F("% HR:"));
    if (hrShown > 0) print3(hrShown); else lcd.print(F("---"));
  }

  lcd.setCursor(0, 1);
  lcd.print(F("Temp:"));
  if (isnan(bodyTemp)) {
    lcd.print(F("--.-"));
  } else {
    if (bodyTemp < 10) lcd.print(' ');
    lcd.print(bodyTemp, 1);
  }
  lcd.print(F("C "));
  lcd.print(alertActive() ? F("ALERT") : F("  OK "));
}

// Called while waiting for sensor samples so temperature and beeping keep running
void housekeeping() {
  updateTemperature();
  updateAlert();
}

// Read one sample from the sensor into position i of the buffers
void readSample(byte i) {
  // getFIFORed/getFIFOIR read the next stored sample immediately. (getRed/getIR wait for a
  // NEW sample on every call, which skipped samples and cut the rate from 25 to about 10 per second.)
  uint32_t r  = particleSensor.getFIFORed();
  uint32_t ir = particleSensor.getFIFOIR();
  lastIR = ir;
  lastRed = r;
  beat.update(ir);
#if DEBUG_RAW
  Serial.println(ir);
#endif
  redBuffer[i] = r  >> SAMPLE_SHIFT;
  irBuffer[i]  = ir >> SAMPLE_SHIFT;
  particleSensor.nextSample();
}

// Median of the last 5 valid SpO2 results (smooths out single bad readings)
void addSpo2(int v) {
  spo2Hist[spo2Pos] = v;
  spo2Pos = (spo2Pos + 1) % 5;
  if (spo2N < 5) spo2N++;
  int t[5];
  for (byte i = 0; i < spo2N; i++) t[i] = spo2Hist[i];
  for (byte i = 1; i < spo2N; i++) {      // insertion sort
    int x = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > x) { t[j + 1] = t[j]; j--; }
    t[j + 1] = x;
  }
  spo2Shown = (spo2N >= 3) ? t[spo2N / 2] : 0;   // show a value only after 3 readings
}

void resetSpo2() {
  spo2N = 0; spo2Pos = 0; spo2Bad = 0; spo2Shown = 0; lowHist = 0;
}

// Decide if a finger is present, run the algorithm and update the results
void evaluate() {
  fingerOn = lastIR > FINGER_THRESHOLD;
  if (fingerOn) {
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer,
                                           &spo2, &validSPO2, &heartRate, &validHeartRate);
    if (beat.bpm > 0) {
      int d = beat.bpm - hrShown; if (d < 0) d = -d;
      if (hrShown == 0 || d * 4 <= hrShown) { hrShown = beat.bpm; hrJump = 0; }  // normal change: accept
      else if (++hrJump >= 12) { hrShown = beat.bpm; hrJump = 0; }               // a big jump must last 12 s
      hrHold = 0;
    }
    else if (hrHold < 5) hrHold++;      // keep the last value for up to 5 seconds
    else { hrShown = 0; hrJump = 0; }
    spo2Ok = validSPO2 && spo2 >= SPO2_MIN_VALID && spo2 <= 100;
    if (spo2Ok) {
      addSpo2(spo2);
      lowHist = (byte)((lowHist << 1) | (spo2 < SPO2_LIMIT ? 1 : 0));
      spo2Bad = 0;
    } else if (++spo2Bad >= 8) {
      resetSpo2();                      // no valid result for 8 s: stop showing an old value
    }
  } else {
    spo2Ok = false;
    resetSpo2();
    hrShown = 0;
    hrHold = 0;
    hrJump = 0;
  }
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.print(F("Health Monitor"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));

  sensors.begin();
  sensors.setWaitForConversion(false);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    lcd.clear();
    lcd.print(F("MAX30102 not"));
    lcd.setCursor(0, 1);
    lcd.print(F("found! Check wire"));
    while (1);
  }

  // LED brightness, sample average, LED mode (2 = red+IR), sample rate, pulse width, ADC range
  particleSensor.setup(LED_BRIGHTNESS, 8, 2, 200, 411, 4096);   // 200 Hz averaged 8x = 25 samples/s

  delay(1500);
  lcd.clear();

  // Collect the first 100 samples
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
      housekeeping();
    }
    readSample(i);
  }
  evaluate();
}

void loop() {
  // Drop the oldest samples, shift the rest down
  for (byte i = NEW_SAMPLES; i < BUFFER_SIZE; i++) {
    redBuffer[i - NEW_SAMPLES] = redBuffer[i];
    irBuffer[i - NEW_SAMPLES]  = irBuffer[i];
  }

  // Take new samples (about 1 second)
  for (byte i = BUFFER_SIZE - NEW_SAMPLES; i < BUFFER_SIZE; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
      housekeeping();
    }
    readSample(i);
  }

  unsigned long now = millis();         // measure the real sample rate (should be about 25 per second)
  if (lastLoopEnd) spsX10 = (unsigned int)((NEW_SAMPLES * 10000UL) / (now - lastLoopEnd));
  lastLoopEnd = now;

  evaluate();
  showReadings();
  updateAlert();

#if DEBUG_IR && !DEBUG_RAW
  Serial.print(F("SPS="));   Serial.print(spsX10 / 10.0, 1); Serial.print(' ');
  Serial.print(F("RED="));   Serial.print(lastRed);
  Serial.print(F(" IR="));   Serial.print(lastIR);
  Serial.print(F(" finger=")); Serial.print(fingerOn);
  Serial.print(F(" SpO2="));  Serial.print(spo2);
  Serial.print(F(" ok="));    Serial.print(spo2Ok);
  Serial.print(F(" HR="));    Serial.print(heartRate);
  Serial.print(F(" BPM="));   Serial.print(hrShown);
  Serial.print(F(" Temp="));  Serial.println(bodyTemp);
#endif
}
