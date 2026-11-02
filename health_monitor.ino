/*
  Smart Health Monitor - Arduino Uno  (version 2)

  Measures SpO2, heart rate (BPM) and body temperature.
  Shows values on a 16x2 I2C LCD. Beeps the buzzer and lights the LED
  when SpO2 < 90% or temperature > 38 C. The alert stops by itself when
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

  HOW TO TUNE
  1. Keep DEBUG_IR = 1 and open the Serial Monitor at 115200 baud.
  2. Note the IR value with NO finger and with a finger on the sensor.
  3. Set FINGER_THRESHOLD between those two numbers.
  4. If IR with a finger is above 65000, lower LED_BRIGHTNESS or set SAMPLE_SHIFT to 1.
  5. If heart rate stays too high (above 120 at rest), lower LED_BRIGHTNESS
     (for example to 30) and rest the finger lightly.
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
#define DEBUG_IR        1        // 1 = print raw IR value to Serial Monitor

const int      SPO2_LIMIT       = 90;      // alert below this (%)
const int      SPO2_MIN_VALID   = 70;      // values below this are treated as noise
const float    TEMP_LIMIT       = 38.0;    // alert above this (C)
const float    TEMP_OFFSET      = 0.8;     // skin-to-body offset; set after comparing with a thermometer
const uint32_t FINGER_THRESHOLD = 5000;   // raw IR below this = no finger (tune with DEBUG_IR)
const byte     LED_BRIGHTNESS   = 100;      // 0-255 (about 0.2 mA per step); lower if readings saturate
const byte     SAMPLE_SHIFT     = 0;       // 0 normally; 1 if IR with a finger is above 65000
const unsigned long BEEP_MS     = 500;     // beep on/off time

// ---------- Objects ----------
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
MAX30105 particleSensor;
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature sensors(&oneWire);

// ---------- Sample buffers (16-bit on Uno to save RAM) ----------
#define BUFFER_SIZE 100
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
bool     fingerOn = false;
bool     spo2Ok = false;      // SpO2 is valid and plausible
byte     lowCount = 0;        // consecutive low-SpO2 results

// ---------- Heart-rate smoothing (average of last 4 valid results) ----------
int  hrHist[4];
byte hrCount = 0, hrIdx = 0;
int  hrShown = 0;

void addHR(int v) {
  hrHist[hrIdx] = v;
  hrIdx = (hrIdx + 1) % 4;
  if (hrCount < 4) hrCount++;
  long s = 0;
  for (byte i = 0; i < hrCount; i++) s += hrHist[i];
  hrShown = s / hrCount;
}

void resetHR() {
  hrCount = 0;
  hrIdx = 0;
  hrShown = 0;
}

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
  bool lowO2 = fingerOn && lowCount >= 2;                 // two low results in a row
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
    if (spo2Ok) print3(spo2); else lcd.print(F("---"));
    lcd.print(F("% HR:"));
    if (hrCount > 0) print3(hrShown); else lcd.print(F("---"));
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
  uint32_t r  = particleSensor.getRed();
  uint32_t ir = particleSensor.getIR();
  lastIR = ir;
  redBuffer[i] = r  >> SAMPLE_SHIFT;
  irBuffer[i]  = ir >> SAMPLE_SHIFT;
  particleSensor.nextSample();
}

// Decide if a finger is present, run the algorithm and update the results
void evaluate() {
  fingerOn = lastIR > FINGER_THRESHOLD;
  if (fingerOn) {
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_SIZE, redBuffer,
                                           &spo2, &validSPO2, &heartRate, &validHeartRate);
    if (validHeartRate && heartRate >= 40 && heartRate <= 180) addHR(heartRate);
    spo2Ok = validSPO2 && spo2 >= SPO2_MIN_VALID && spo2 <= 100;
    if (spo2Ok && spo2 < SPO2_LIMIT) {
      if (lowCount < 255) lowCount++;
    } else {
      lowCount = 0;
    }
  } else {
    spo2Ok = false;
    lowCount = 0;
    resetHR();
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
  particleSensor.setup(LED_BRIGHTNESS, 4, 2, 100, 411, 4096);

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
  // Drop the oldest 25 samples, shift the rest down
  for (byte i = 25; i < BUFFER_SIZE; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }

  // Take 25 new samples (about 1 second)
  for (byte i = 75; i < BUFFER_SIZE; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
      housekeeping();
    }
    readSample(i);
  }

  evaluate();
  showReadings();
  updateAlert();

#if DEBUG_IR
  Serial.print(F("IR="));    Serial.print(lastIR);
  Serial.print(F(" finger=")); Serial.print(fingerOn);
  Serial.print(F(" SpO2="));  Serial.print(spo2);
  Serial.print(F(" ok="));    Serial.print(spo2Ok);
  Serial.print(F(" HR="));    Serial.print(heartRate);
  Serial.print(F(" avgHR=")); Serial.print(hrShown);
  Serial.print(F(" Temp="));  Serial.println(bodyTemp);
#endif
}
