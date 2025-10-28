#include <Servo.h>

/* ---------- PIN CONFIG ---------- */
const int TRIG_PIN    = 3;
const int ECHO_PIN    = 4;
const int BUTTON_PIN  = 12;  // Pushbutton wired to GND, use INPUT_PULLUP
const int BUZZER_PIN  = 6;
const int LED_PIN     = 5;
const int GSM_LED     = 7;
const int SERVO_PIN   = 9;

/* ---------- PARAMETERS (tune as needed) ---------- */
const int OPEN_ANGLE      = 0;    // servo angle = gate OPEN (adjust to match your arm)
const int CLOSED_ANGLE    = 30;    // servo angle = gate CLOSED
const int DETECT_DIST_CM  = 50;    // distance threshold to detect approaching "train" (cm)
const int CLEAR_DIST_CM   = 70;    // distance considered "train passed" (cm)
const unsigned long AUTO_CLOSE_MS = 5000UL;  // auto-close timeout if no manual response
const unsigned long AUTO_OPEN_MS  = 5000UL;  // auto-open after train passed
const unsigned long DEBOUNCE_MS   = 40UL;    // button debounce

/* ---------- GLOBALS ---------- */
Servo gateServo;
bool gateIsClosed       = false;
bool alertsActive       = false;
bool trainDetected      = false;
bool waitingToOpen      = false;

unsigned long alertStartMillis = 0;
unsigned long trainClearMillis = 0;

/* Debounce state */
int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

/* ---------- FUNCTIONS ---------- */
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30 ms timeout
  if (duration == 0UL) return -1.0;
  return (duration * 0.034) / 2.0;
}

/* returns true only when button transitions to PRESSED (edge) */
bool buttonPressedEvent() {
  int reading = digitalRead(BUTTON_PIN); // HIGH when released (INPUT_PULLUP)
  if (reading != lastButtonReading) lastDebounceTime = millis();
  lastButtonReading = reading;
  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == LOW) return true; // pressed event
    }
  }
  return false;
}

void startAlerts() {
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(GSM_LED, HIGH);
  alertsActive = true;
  Serial.println(F("Alerts ON"));
}

void stopAlerts() {
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(GSM_LED, LOW);
  alertsActive = false;
  Serial.println(F("Alerts OFF"));
}

void closeGateManual() {
  gateServo.write(CLOSED_ANGLE);
  gateIsClosed = true;
  stopAlerts();               // stop buzzer/led immediately
  waitingToOpen = false;      // prevent auto-open sequence until train clears
  Serial.println(F("Gate CLOSED manually by gatekeeper"));
}

void closeGateAuto() {
  gateServo.write(CLOSED_ANGLE);
  gateIsClosed = true;
  stopAlerts();
  waitingToOpen = false;
  Serial.println(F("Gate CLOSED automatically (no response)"));
}

void openGate() {
  gateServo.write(OPEN_ANGLE);
  gateIsClosed = false;
  stopAlerts();
  waitingToOpen = false;
  trainDetected = false;
  Serial.println(F("Gate OPEN (safe to cross)"));
}

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(GSM_LED, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(GSM_LED, LOW);

  gateServo.attach(SERVO_PIN);
  gateServo.write(OPEN_ANGLE);
  gateIsClosed = false;
  Serial.println(F("System Ready - Gate OPEN"));
}

/* ---------- MAIN LOOP ---------- */
void loop() {
  float distance = readDistanceCm();           // -1.0 if no echo
  bool pressEvent = buttonPressedEvent();      // true only on new press (edge)

  // debug
  Serial.print(F("Dist(cm): "));
  if (distance < 0) Serial.print(F("no_read"));
  else Serial.print(distance);
  Serial.print(F(" | GateClosed: "));
  Serial.print(gateIsClosed ? "YES" : "NO");
  Serial.print(F(" | Alerts: "));
  Serial.println(alertsActive ? "ON" : "OFF");

  // 1) If gate is OPEN and we detect an approaching 'train' → start alerts
  if (!gateIsClosed && distance > 0 && distance <= DETECT_DIST_CM && !alertsActive) {
    startAlerts();
    alertStartMillis = millis();
    trainDetected = true;
  }

  // 2) While alerts are active → allow manual close or auto-close after timeout
  if (alertsActive) {
    if (pressEvent) {
      // gatekeeper responded → manual close
      closeGateManual();
    } else if (millis() - alertStartMillis >= AUTO_CLOSE_MS) {
      // no manual response → auto close
      closeGateAuto();
    }
    // (pressEvent consumed here; it will NOT be used for reopen in this iteration)
  }
  else {
    // alerts NOT active -> only consider manual reopen when safe
    if (gateIsClosed && pressEvent && (distance < 0 || distance >= CLEAR_DIST_CM)) {
      // manual reopen by gatekeeper when no train nearby
      openGate();
      Serial.println(F("Gate reopened manually by gatekeeper"));
    }
  }

  // 3) When gate is closed and train passes (distance becomes large) -> schedule auto reopen
  if (trainDetected && gateIsClosed && distance >= CLEAR_DIST_CM && !waitingToOpen) {
    waitingToOpen = true;
    trainClearMillis = millis();
    Serial.println(F("Train passed. Will auto-open after delay"));
  }

  // 4) Auto-open after train-clear delay
  if (waitingToOpen && (millis() - trainClearMillis >= AUTO_OPEN_MS)) {
    openGate();
  }

  delay(100); // small delay for stability
}
