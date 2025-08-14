#include <Wire.h>
#include <math.h>
#include <avr/wdt.h>
#include "RichShieldDHT.h"
#include "RichShieldTM1637.h"
#include "RichShieldLightSensor.h"
#include "RichShieldPassiveBuzzer.h"

// --- Pin Definitions ---
#define LED_RED     4
#define LED_GREEN   5
#define LED_BLUE    6
#define LED_YELLOW  7

#define K2          8
#define K1          9

#define CLK         10   // TM1637 Clock
#define DIO         11   // TM1637 Data

#define DHT11_SEN   12
#define BUZZER      3
#define LIGHTSENSOR_PIN A2

// --- Object Instantiations ---
TM1637 disp(CLK, DIO);
PassiveBuzzer buz(BUZZER);
DHT dht(DHT11_SEN);
LightSensor lightsensor(LIGHTSENSOR_PIN); // LDR sensor

// --- Global Variables ---
float tempVals[5] = {0, 0, 0, 0, 0};
float humiVals[5] = {0, 0, 0, 0, 0};
float Temperature_READ = 0;
float Humidity_READ = 0;
int tempIndex = 0;
int humiIndex = 0;
int HIGH_LOW_TEMP_EVENT = 0;
int HIGH_LOW_HUMI_EVENT = 0;
int tempCount = 0;
int humiCount = 0;
unsigned long blueLEDCycleStart = 0;
int FULL_PANIC_RST_EN = 0;
int MEDIUM = 0;

// --- Min/Max Tracking Arrays ---
float MAXrep[3] = {-999.0, -999.0, -999.0}; // [0]=temp, [1]=humidity, [2]=ldr
float MINrep[3] = {999.0, 999.0, 999.0};   // [0]=temp, [1]=humidity, [2]=ldr

// --- Function to get parameter status ---
int getParameterStatus() {
  float lux = LDR() / 4;

  bool tempOutOfRange = (Temperature_READ < 24 || Temperature_READ > 34) || (HIGH_LOW_TEMP_EVENT == 1);
  bool humiOutOfRange = (Humidity_READ < 60 || Humidity_READ > 80) || (HIGH_LOW_HUMI_EVENT == 1);
  bool ldrOutOfRange  = (lux < 21 || lux > 300);

  int outOfRangeCount = 0;
  int lastOutOfRange = 17; // blank

  if (humiOutOfRange) { outOfRangeCount++; lastOutOfRange = 1; }
  if (tempOutOfRange) { outOfRangeCount++; lastOutOfRange = 2; }
  if (ldrOutOfRange)  { outOfRangeCount++; lastOutOfRange = 3; }

  if (outOfRangeCount > 1) return 4; // Multiple parameters
  if (outOfRangeCount == 1) return lastOutOfRange;
  return 17; // All good, blank
}

// --- Function to update min/max values ---
void updateMinMaxValues() {
  float lux = LDR() / 4;

  // Update min/max values - first reading initializes properly
  if (MAXrep[0] == -999.0 || Temperature_READ > MAXrep[0]) MAXrep[0] = Temperature_READ;
  if (MINrep[0] == 999.0 || Temperature_READ < MINrep[0]) MINrep[0] = Temperature_READ;
  if (MAXrep[1] == -999.0 || Humidity_READ > MAXrep[1]) MAXrep[1] = Humidity_READ;
  if (MINrep[1] == 999.0 || Humidity_READ < MINrep[1]) MINrep[1] = Humidity_READ;
  if (MAXrep[2] == -999.0 || lux > MAXrep[2]) MAXrep[2] = lux;
  if (MINrep[2] == 999.0 || lux < MINrep[2]) MINrep[2] = lux;
}

unsigned long warningLEDTimestamp = 0;
bool warningLEDState = false;
int warningLEDPin = -1;
unsigned long warningLEDInterval = 500;
bool warningLEDActive = false;

// --- Panic and LED control ---
bool buzzerPlayed = false;
bool greenBlinkDone = false;
unsigned long lastBlinkTime = 0;
bool yellowBlinkState = false;
int greenBlinkCount = 0;



void setup() {
  Serial.begin(9600);

  disp.init();          // Initialize 7-segment display
  dht.begin();          // Start DHT sensor

  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  pinMode(K1, INPUT_PULLUP);
  pinMode(K2, INPUT_PULLUP);


}

void loop() {
  readSerialCommands();

  Temp();
  delay(500);
  Humi();

  float lux = LDR() / 4;

  // Update min/max values in background
  updateMinMaxValues();

  Serial.print("Temperature:");
  Serial.print(Temperature_READ);
  Serial.print("\tHumidity:");
  Serial.print(Humidity_READ);
  Serial.print("\tLux:");
  Serial.println(lux);

  // --- Evaluate Conditions ---
  // No, MEDIUM VALS TRIG doesn't trigger in PANIC, this is a feature, not a bug.
  // PANIC is not to be overridden, it is important as a warning. DO NOT ATTEMPT TO OVERRIDE PANIC.
  // MEDIUM VALS TRIG via serial monitor only triggers in GOOD or already existing Meh conditions!
  bool tempPanic = (Temperature_READ < 23 || Temperature_READ > 36);
  bool humiPanic = (Humidity_READ < 50 || Humidity_READ > 75);
  bool ldrPanic  = (lux < 5 || lux > 300);

  bool tempGood = (Temperature_READ >= 24 && Temperature_READ <= 34);
  bool humiGood = (Humidity_READ >= 60 && Humidity_READ <= 80);
  bool ldrGood  = (lux >= 21 && lux <= 300);

  bool tempMeh = !tempPanic && !tempGood;
  bool humiMeh = !humiPanic && !humiGood;
  bool ldrMeh  = !ldrPanic && !ldrGood;

  bool isPanic = tempPanic || humiPanic || ldrPanic;
  bool isMeh   = (!isPanic && (tempMeh || humiMeh || ldrMeh)) || (MEDIUM == 1);
  bool isGood  = tempGood && humiGood && ldrGood && (MEDIUM == 0);

  handleSystemState(isPanic, isMeh, isGood);

  BlueLED_Timer();
  updateWarningLED();

  delay(400);
}

void handleSystemState(bool isPanic, bool isMeh, bool isGood) {
  if (isPanic) {
    // --- PANIC State ---
    FULL_PANIC_RST_EN = 1;
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, HIGH);

    startWarningLED(LED_RED, 300); // RED blinking warning LED

    if (!buzzerPlayed) {
      warning_buzzer();   // Play buzzer once
      buzzerPlayed = true;
    }

    greenBlinkDone = false;
  }
  else if (isMeh) {
    // --- MEH State ---
    FULL_PANIC_RST_EN = 0;
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    stopWarningLED();

    // Blink Yellow LED
    unsigned long now = millis();
    if (now - lastBlinkTime > 500) {
      lastBlinkTime = now;
      yellowBlinkState = !yellowBlinkState;
      digitalWrite(LED_YELLOW, yellowBlinkState ? HIGH : LOW);
    }

    buzzerPlayed = false;
    greenBlinkDone = false;
  }
  else if (isGood) {
    // --- GOOD State ---
    FULL_PANIC_RST_EN = 0;
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YELLOW, LOW);
    stopWarningLED();
    buzzerPlayed = false;

    if (!greenBlinkDone) {
      // Blink green 3 times before keeping it on
      unsigned long now = millis();
      if (now - lastBlinkTime > 300) {
        lastBlinkTime = now;
        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
        if (digitalRead(LED_GREEN)) {
          greenBlinkCount++;
        }
        if (greenBlinkCount >= 3) {
          greenBlinkDone = true;
          greenBlinkCount = 0;
          digitalWrite(LED_GREEN, HIGH);
        }
      }
    } else {
      digitalWrite(LED_GREEN, HIGH);
    }
  }
}

// --- Non-blocking Warning LED Functions ---
void startWarningLED(int pin, unsigned long interval) {
  warningLEDPin = pin;
  warningLEDInterval = interval;
  warningLEDTimestamp = millis();
  warningLEDState = false;
  warningLEDActive = true;
  pinMode(pin, OUTPUT);
}

void stopWarningLED() {
  if (warningLEDPin != -1) digitalWrite(warningLEDPin, LOW);
  warningLEDActive = false;
}

void updateWarningLED() {
  if (!warningLEDActive) return;
  unsigned long currentMillis = millis();
  if (currentMillis - warningLEDTimestamp >= warningLEDInterval) {
    warningLEDTimestamp = currentMillis;
    warningLEDState = !warningLEDState;
    digitalWrite(warningLEDPin, warningLEDState ? HIGH : LOW);
  }
}

void warning_buzzer() {
  int melody[] = {659, 784, 698, 880, 784};
  int noteDuration[] = {200, 200, 200, 200, 300};

  for (int i = 0; i < 5; i++) {
    buz.playTone(melody[i], noteDuration[i]);
    delay(50);
  }
}

void Humi() {
  if (humiCount < 5) {
    for (int i = 0; i < 5; i++) {
      humiVals[i] = dht.readHumidity();
      delay(50);
    }
    humiCount = 5;
  } else {
    for (int i = 0; i < 4; i++) humiVals[i] = humiVals[i + 1];
    humiVals[4] = dht.readHumidity();
  }

  float sum = 0;
  for (int i = 0; i < 5; i++) sum += humiVals[i];
  int avg = int(sum / 5);

  Humidity_READ = avg;

  disp.display(0, (avg / 10) % 10);
  disp.display(1, avg % 10);
  disp.display(2, 18);  // 'H'
  disp.display(3, getParameterStatus());
  delay(1000);
}

void Temp() {
  if (tempCount < 5) {
    for (int i = 0; i < 5; i++) {
      tempVals[i] = dht.readTemperature();
      delay(50);
    }
    tempCount = 5;
  } else {
    for (int i = 0; i < 4; i++) tempVals[i] = tempVals[i + 1];
    tempVals[4] = dht.readTemperature();
  }

  float sum = 0;
  for (int i = 0; i < 5; i++) sum += tempVals[i];
  int avg = int(sum / 5);

  Temperature_READ = avg;

  disp.display(0, (avg / 10) % 10);
  disp.display(1, avg % 10);
  disp.display(2, 12);  // 'C'
  disp.display(3, getParameterStatus());
  delay(1000);
}

float LDR() {
  float Rsensor = lightsensor.getRes();
  float lux = 325 * pow(Rsensor, -1.4);
  return lux;
}

void Brightness_Blue() {
  int knobValue = analogRead(A0);
  int brightness = map(knobValue, 0, 1023, 0, 255);
  analogWrite(LED_BLUE, brightness);
}

void readSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "HIGH_LOW_TEMP_EVENT 1") {
      HIGH_LOW_TEMP_EVENT = 1;
      Serial.println("HIGH_LOW_TEMP_EVENT set to 1");

    } else if (command == "HIGH_LOW_TEMP_EVENT 0") {
      HIGH_LOW_TEMP_EVENT = 0;
      Serial.println("HIGH_LOW_TEMP_EVENT set to 0");

    } else if (command == "HIGH_LOW_HUMI_EVENT 1") {
      HIGH_LOW_HUMI_EVENT = 1;
      Serial.println("HIGH_LOW_HUMI_EVENT set to 1");

    } else if (command == "HIGH_LOW_HUMI_EVENT 0") {
      HIGH_LOW_HUMI_EVENT = 0;
      Serial.println("HIGH_LOW_HUMI_EVENT set to 0");

    } else if (command == "WARNING_LED ON") {
      startWarningLED(LED_YELLOW, 500);
      Serial.println("Warning LED started.");

    } else if (command == "WARNING_LED OFF") {
      stopWarningLED();
      Serial.println("Warning LED stopped.");

    } else if (command == "MEDIUM VALS TRIG") {
      MEDIUM = 1;
      Serial.println("MEH override activated (MEDIUM = 1)");

    } else if (command == "CLEAR MEDIUM") {
      MEDIUM = 0;
      Serial.println("MEH override cleared (MEDIUM = 0)");

    } else if (command == "RESET PANIC") {
      Serial.println("Resetting PANIC state...");
      HIGH_LOW_TEMP_EVENT = 0;
      HIGH_LOW_HUMI_EVENT = 0;
      FULL_PANIC_RST_EN = 0;
      MEDIUM = 0;
      stopWarningLED();
      buzzerPlayed = false;
      greenBlinkDone = false;
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_GREEN, LOW);  // Will be updated next loop

    } else if (command == "MINMAX") {
      Serial.print("Min/Max - Temp: "); Serial.print(MINrep[0]);
      Serial.print("/"); Serial.print(MAXrep[0]);
      Serial.print("°C, Humid: "); Serial.print(MINrep[1]);
      Serial.print("/"); Serial.print(MAXrep[1]);
      Serial.print("%, Light: "); Serial.print(MINrep[2]);
      Serial.print("/"); Serial.print(MAXrep[2]); Serial.println(" lux");

    } else {
      Serial.println("Unknown command.");
    }
  }
}

void BlueLED_Timer() {
  unsigned long now = millis();
  unsigned long elapsed = now - blueLEDCycleStart;

  const unsigned long ON_DURATION = 18UL * 60UL * 60UL * 1000UL;  // 18 hours
  const unsigned long OFF_DURATION = 6UL * 60UL * 60UL * 1000UL;  // 6 hours
  const unsigned long CYCLE_DURATION = ON_DURATION + OFF_DURATION;

  if (elapsed >= CYCLE_DURATION) {
    blueLEDCycleStart = now;
    elapsed = 0;
  }

  if (elapsed < ON_DURATION) {
    Brightness_Blue();
  } else {
    analogWrite(LED_BLUE, 0);
  }
}

// TEST OR ABSOLUTELY DO NOT USE THIS. I was tired.
void reboot() {
  cli();             // Disable interrupts
  wdt_reset();       // Reset watchdog timer
  MCUSR &= ~(1 << WDRF); // Clear watchdog reset flag
  wdt_disable();
  wdt_enable(WDTO_15MS); // Enable with 15ms timeout
  sei();             // Re-enable interrupts
  while (1) {}       // Wait for reset
}
