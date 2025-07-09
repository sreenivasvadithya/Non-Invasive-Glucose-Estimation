#include <WiFi.h>
#include <HTTPClient.h>

/* Wi-Fi Credentials */
const char* ssid = "sreenivas";
const char* password = "sreenivas194";

/* Firebase Configuration */
const String firebaseHost = "https://glucosemonitor-77d2a-default-rtdb.asia-southeast1.firebasedatabase.app/";
const String firebasePath = "/readings.json"; // Path to push readings

/* Hardware Configuration */
const int irLED = 12;
const int pdPin = 34;
const int greenLED = 25; 
const int redLED = 26;
const float vRef = 3.3;
const int adcResolution = 4095;

/* Glucose Thresholds (adjust these based on your calibration) */
const float lowGlucoseThreshold = 0.120;    // Below this - no LED
const float normalGlucoseThreshold = 1.800; // 0.270-1.800 is normal (green)
                                           // Above 1.800 is high (red)

/* Measurement Parameters */
const int calibrationSamples = 100;
const int measurementSamples = 20;
const int sampleDelay = 50;
const float minValidVoltage = 0.05;
const float maxValidVoltage = 3.2;

/* Finger Detection Parameters */
bool fingerPresent = false;
unsigned long lastDetectionTime = 0;
const unsigned long detectionTimeout = 5000;
const unsigned long debounceTime = 300;
int fingerThreshold = 100;

/* LED Blink Parameters */
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 500; // Blink every 500ms

/* Filtering Variables */ 
float irHistory[5] = {0};
int historyIndex = 0;

void setup() {
  pinMode(irLED, OUTPUT);
  pinMode(greenLED, OUTPUT);  // Initialize LED's
  pinMode(redLED, OUTPUT);    
  digitalWrite(irLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");

  Serial.println("System Initializing...");
  Serial.println("Calibration - keep finger away from sensor");
  delay(2000);
  calibrateSensor();

  Serial.println("\nSystem Ready. Format: IR_Voltage(V),Status");
  Serial.println("Header: IR_V,Status");
}

void loop() {
  bool currentFingerState = checkFingerPresence();
  
  if (currentFingerState != fingerPresent) {
    if (millis() - lastDetectionTime > debounceTime) {
      fingerPresent = currentFingerState;
      lastDetectionTime = millis();
      if (fingerPresent) {
        Serial.println("Finger detected - starting measurements");
      } else {
        Serial.println("Finger removed - stopping measurements");
        // Turn off both LEDs when finger is removed
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, LOW);
      }
    }
  } else {
    lastDetectionTime = millis();
  }

  if (fingerPresent) {
    takeMeasurement();
    delay(800);
  } else {
    delay(200);
  }

  if (fingerPresent && (millis() - lastDetectionTime > detectionTimeout)) {
    fingerPresent = false;
    Serial.println("Timeout - finger assumed removed");
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
  }
}

void calibrateSensor() {
  long ambientSum = 0;
  long irSum = 0;

  for (int i = 0; i < calibrationSamples; i++) {
    digitalWrite(irLED, LOW);
    ambientSum += analogRead(pdPin);
    delay(10);
  }

  for (int i = 0; i < calibrationSamples; i++) {
    digitalWrite(irLED, HIGH);
    delayMicroseconds(500);  // Increased pulse width
    irSum += analogRead(pdPin);
    digitalWrite(irLED, LOW);
    delay(10);
  }

  int ambientAvg = ambientSum / calibrationSamples;
  int irAvg = irSum / calibrationSamples;
  fingerThreshold = (irAvg - ambientAvg) * 0.3;  // threshold multiplier
  if (fingerThreshold < 100) fingerThreshold = 100;  // Higher minimum threshold

  Serial.print("Calibration Results - Ambient: ");
  Serial.print(ambientAvg);
  Serial.print(", IR Response: ");
  Serial.print(irAvg);
  Serial.print(", Detection Threshold: ");
  Serial.println(fingerThreshold);
}

bool checkFingerPresence() {
  int ambientSum = 0;
  int irActiveSum = 0;

  for (int i = 0; i < 5; i++) {
    digitalWrite(irLED, LOW);
    delayMicroseconds(500);
    ambientSum += analogRead(pdPin);

    digitalWrite(irLED, HIGH);
    delayMicroseconds(500);  // pulse width
    irActiveSum += analogRead(pdPin);
    digitalWrite(irLED, LOW);

    delay(10);
  }

  int ambientAvg = ambientSum / 5;
  int irActiveAvg = irActiveSum / 5;

  return (irActiveAvg - ambientAvg) > fingerThreshold;
}

float adcToVoltage(int adcValue) {
  return (adcValue * vRef) / adcResolution;
}

void updateLEDs(float voltage, String status) {
  // First turn off both LEDs
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  
  // Check if it's time to blink
  if (millis() - lastBlinkTime > blinkInterval) {
    lastBlinkTime = millis();
    
    if (voltage < lowGlucoseThreshold) {
      // Below threshold - no LED
      digitalWrite(greenLED, LOW);
      digitalWrite(redLED, LOW);
    } 
    else if (voltage >= lowGlucoseThreshold && voltage <= normalGlucoseThreshold) {
      // Normal glucose level - blink green
      digitalWrite(greenLED, !digitalRead(greenLED));
      digitalWrite(redLED, LOW);
    } 
    else if (voltage > normalGlucoseThreshold) {
      // High glucose level - blink red
      digitalWrite(redLED, !digitalRead(redLED));
      digitalWrite(greenLED, LOW);
    }
  }
}

void takeMeasurement() {
  float irSum = 0;
  int validSamples = 0;

  for (int i = 0; i < measurementSamples; i++) {
    digitalWrite(irLED, HIGH);
    delayMicroseconds(500);  // Increased pulse width
    float irVoltage = adcToVoltage(analogRead(pdPin));
    digitalWrite(irLED, LOW);

    if (irVoltage > minValidVoltage && irVoltage < maxValidVoltage) {
      irSum += irVoltage;
      validSamples++;
    }

    delay(sampleDelay);
  }

  if (validSamples < measurementSamples / 2) {
    Serial.println("0,INVALID_READINGS");
    updateLEDs(0, "INVALID");
    return;
  }

  float irAvg = irSum / validSamples;

  // Apply moving average filter
  irHistory[historyIndex] = irAvg;
  historyIndex = (historyIndex + 1) % 5;

  float irFiltered = 0;
  for (int i = 0; i < 5; i++) {
    irFiltered += irHistory[i];
  }
  irFiltered /= 5;

  String status = "OK";
  if (irFiltered >= maxValidVoltage) {
    status = "SATURATED";
  } else if (irFiltered <= minValidVoltage) {
    status = "NO_SIGNAL";
  }

  Serial.print(irFiltered, 4);
  Serial.print(",");
  Serial.println(status);

  // Update LEDs based on reading
  updateLEDs(irFiltered, status);

  // SEND TO FIREBASE
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = firebaseHost + firebasePath;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    String jsonData = "{\"IR_Voltage\": " + String(irFiltered, 4) + ", \"Status\": \"" + status + "\"}";
    int httpResponseCode = http.POST(jsonData);
    Serial.print("Firebase Response: ");
    Serial.println(httpResponseCode);

    http.end();
  } else {
    Serial.println("WiFi not connected!");
  }
}
