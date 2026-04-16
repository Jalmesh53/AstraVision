#include <Servo.h>

// --- PINS ---
#define enA 11
#define in1 9
#define in2 8

#define enB 5
#define in3 7
#define in4 6

#define L_S A0
#define R_S A1

#define trigger A2
#define echo A3

#define triggerBack A4
#define echoBack A5

#define SERVO_PIN 10
#define BUZZER_PIN 12

// --- PARAMETERS ---
const int MOTOR_SPEED = 80;
const int STOP_DISTANCE = 50;
const int ENTER_DISTANCE = 45;   // slightly less than STOP_DISTANCE

// --- STATE ---
bool rearMode = false;
unsigned long rearStartTime = 0;

Servo scanServo;

// --- ULTRASONIC STATE ---
unsigned long lastPingTime = 0;
int frontDist = 400;
int backDist = 400;
int prevFrontDist = 400;
int prevBackDist = 400;
bool frontWasBlockedAtStart = false;
int rearStartBackDist = 400;
bool rearCleared = false;

// --- NON-BLOCKING DISTANCE ---
int getDistance(int trig, int ech) {

  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long start = micros();

  while (digitalRead(ech) == LOW) {
    if (micros() - start > 10000) return 400;
  }

  unsigned long echoStart = micros();

  while (digitalRead(ech) == HIGH) {
    if (micros() - echoStart > 10000) return 400;
  }

  unsigned long duration = micros() - echoStart;

  return duration * 0.034 / 2;
}

void setup() {

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(L_S, INPUT);
  pinMode(R_S, INPUT);

  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(triggerBack, OUTPUT);
  pinMode(echoBack, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(0);
}

void loop() {

  // --- UPDATE DISTANCE EVERY 60 ms ---
  if (millis() - lastPingTime > 60) {

  frontDist = getDistance(trigger, echo);
  backDist  = getDistance(triggerBack, echoBack);

  lastPingTime = millis();
}

  // --- ENTER REAR MODE ---
  if (!rearMode && backDist <= ENTER_DISTANCE) {

    rearCleared = false;
    prevBackDist = backDist;
    rearStartBackDist = backDist;
    rearMode = true;
    rearStartTime = millis();

    frontWasBlockedAtStart = (frontDist <= STOP_DISTANCE && frontDist > 0);
    prevFrontDist = frontDist;

    stopMotors();
    digitalWrite(BUZZER_PIN, HIGH);

    scanServo.write(180);
    delay(300);
  }

  // --- REAR MODE ---
  if (rearMode) {

  stopMotors();
  digitalWrite(BUZZER_PIN, HIGH);

  unsigned long elapsed = millis() - rearStartTime;

  // --- ALWAYS READ FRONT SENSOR ---
  frontDist = getDistance(trigger, echo);
  backDist  = getDistance(triggerBack, echoBack);

  // --- PHASE 1: INITIAL STABLE TIME ---
  if (elapsed < 400) {
    return;
  }

  // --- PHASE 2: 0–3 SEC ---
  if (elapsed < 3000) {

  // 🔥 FORCE ignore frontDist completely
  frontDist = 400;

  // Step 1: wait until rear becomes clear once
  if (!rearCleared) {
    if (backDist > STOP_DISTANCE) {
      rearCleared = true;
    }
    return;
  }

  // Step 2: detect rear
  if (backDist <= (STOP_DISTANCE - 5) && backDist > 0) {
    rearMode = false;
    scanServo.write(0);
    delay(200);
    return;
  }

  return;
}

  // --- PHASE 3: AFTER 3 SEC ---

  // 🔴 if still obstacle → keep waiting
  if (frontDist <= STOP_DISTANCE && frontDist > 0) {
    return;
  }

  // 🟢 clear → exit
  rearMode = false;
  digitalWrite(BUZZER_PIN, LOW);
  scanServo.write(0);

  return;
}

  // --- FRONT OBSTACLE ---
  if (frontDist <= STOP_DISTANCE && frontDist > 0) {

    stopMotors();
    digitalWrite(BUZZER_PIN, HIGH);
    return;
  }

  digitalWrite(BUZZER_PIN, LOW);

  // --- LINE FOLLOWING ---
  int left = digitalRead(L_S);
  int right = digitalRead(R_S);

  if (left == LOW && right == LOW) {
    goForward();
  }
  else if (left == HIGH && right == LOW) {
    turnRight();
  }
  else if (left == LOW && right == HIGH) {
    turnLeft();
  }
  else {
    stopMotors();
  }
}

// --- MOTOR CONTROL ---
void goForward() {
  analogWrite(enA, MOTOR_SPEED);
  analogWrite(enB, MOTOR_SPEED);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnLeft() {
  analogWrite(enA, MOTOR_SPEED);
  analogWrite(enB, MOTOR_SPEED);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight() {
  analogWrite(enA, MOTOR_SPEED);
  analogWrite(enB, MOTOR_SPEED);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}