/* 

Robot Car with Bluetooth + Voice + Obstacle Avoidance

*/

#include <SoftwareSerial.h>
#include <Servo.h>

// Bluetooth Serial
SoftwareSerial BT_Serial(2, 3); // RX, TX

// Motor Driver pins
#define enA 11
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5

// Ultrasonic + Servo
const uint8_t servoPin   = A4;
const uint8_t triggerPin = A2;
const uint8_t echoPin    = A3;

// Tunable settings
const int OBSTACLE_DISTANCE_CM      = 20;
const unsigned long BUTTON_TIMEOUT_MS = 2000UL;
const unsigned long SERVO_SETTLE_MS   = 80UL;

// Servo angles (0–180)
const int NEUTRAL_ANGLE  = 180;  // default forward-ish
const int FORWARD_ANGLE  = 180;
const int BACKWARD_ANGLE = 0;
const int LEFT_ANGLE     = 180;
const int RIGHT_ANGLE    = 180;

// Globals
Servo myServo;
int Speed = 220;

unsigned long lastButtonMillis = 0;
bool  buttonControlActive = false;
bool  movingForward       = false;

String btBuffer = "";

int currentServoAngle  = NEUTRAL_ANGLE;  // where servo is now
int desiredServoAngle  = NEUTRAL_ANGLE;  // where we want servo to be

// ================= SETUP =====================
void setup() {
  Serial.begin(9600);
  BT_Serial.begin(9600);

  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(12, OUTPUT);

  // Ultrasonic pins
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo
  myServo.attach(servoPin);
  myServo.write(NEUTRAL_ANGLE);
  currentServoAngle = NEUTRAL_ANGLE;
  desiredServoAngle = NEUTRAL_ANGLE;
  delay(200);

  analogWrite(enA, Speed);
  analogWrite(enB, Speed);

  Serial.println("System Ready – Waiting for Bluetooth commands...");
}

// ================= MAIN LOOP =====================
void loop() {
  handleBTInput();

  // check obstacle in the direction we are heading
if (!isPathClear()) {
    Stop();
    Serial.println("Obstacle detected → Forced Stop");
    delay(100);
}


  // Keep motors powered for button mode
  if (buttonControlActive && (millis() - lastButtonMillis <= BUTTON_TIMEOUT_MS)) {
    analogWrite(enA, Speed);
    analogWrite(enB, Speed);
  } else {
    buttonControlActive = false;
  }

  // Apply servo movement once per loop
  updateServo();
}

// ================= SERVO UPDATE =====================
void updateServo() {
  int angle = constrain(desiredServoAngle, 0, 180);
  if (angle == currentServoAngle) return;

  myServo.write(angle);
  delay(SERVO_SETTLE_MS);
  currentServoAngle = angle;

  Serial.print("Servo moved to: ");
  Serial.println(currentServoAngle);
}

// ================= BLUETOOTH INPUT =====================
void handleBTInput() {
  while (BT_Serial.available() > 0) {
    char c = BT_Serial.read();
    Serial.print("RX raw: '"); Serial.print(c); Serial.println("'");

    if (c == '\n' || c == '\r' || c == ' ') {
      continue; // ignore separators
    }

    char first = tolower(c);
    if (isMovementButton(first)) {
      Serial.print("Detected movement-first-char: ");
      Serial.println(first);

      // Flush rest of word (non-blocking)
      delay(10);
      while (BT_Serial.available()) { BT_Serial.read(); }

      processButtonInput(first);
      return;
    }

    // Buffer for voice commands (if app sends lines)
    btBuffer += c;
    if (c == '\n' || c == '\r') {
      btBuffer.trim();
      if (btBuffer.length() > 0) {
        Serial.print("Voice Command: ");
        Serial.println(btBuffer);
        processVoiceCommand(btBuffer);
      }
      btBuffer = "";
    }
  }
}

bool isMovementButton(char c) {
  return (c=='f' || c=='b' || c=='l' || c=='r' || c=='s');
}

void processButtonInput(char c) {
  c = tolower(c);
  lastButtonMillis   = millis();
  buttonControlActive = true;

  Serial.print("Button Command: ");
  Serial.println(c);

  if (c=='f')      forward();
  else if (c=='b') backward();
  else if (c=='l') left();
  else if (c=='r') right();
  else if (c=='s') Stop();

  analogWrite(enA, Speed);
  analogWrite(enB, Speed);
}

// ================= VOICE COMMANDS =====================
void processVoiceCommand(String cmd) {
  cmd.toLowerCase();

  if (cmd.startsWith("speed")) {
    int value = cmd.substring(6).toInt();
    Speed = constrain(value, 0, 255);
    analogWrite(enA, Speed);
    analogWrite(enB, Speed);
    Serial.print("Speed Set to: ");
    Serial.println(Speed);
  }
  else if (cmd=="forward")         forward();
  else if (cmd=="back" || cmd=="backward") backward();
  else if (cmd=="left")           left();
  else if (cmd=="right")          right();
  else if (cmd=="stop")           Stop();
  else                            Serial.println("Unknown Voice Command");
}

// ================= MOTOR CONTROL =====================
void forward() {
  Serial.println("Forward");
  desiredServoAngle = FORWARD_ANGLE;
  movingForward     = true;

  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void backward() {
  Serial.println("Backward");
  desiredServoAngle = BACKWARD_ANGLE;
  movingForward     = false;

  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  digitalWrite(12, HIGH);
}

void left() {
  Serial.println("Left");
  desiredServoAngle = LEFT_ANGLE;
  movingForward     = false;

  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void right() {
  Serial.println("Right");
  desiredServoAngle = RIGHT_ANGLE;
  movingForward     = false;

  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
}

void Stop() {
  Serial.println("STOP");
  movingForward = false;

  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

// ================= ULTRASONIC CHECK =====================
bool isPathClear() {
  
  // Use the direction we are currently moving in
  int checkAngle = desiredServoAngle;

  // Temporarily point servo to direction of motion
  int savedAngle = currentServoAngle;
  if (savedAngle != checkAngle) {
    myServo.write(checkAngle);
    delay(SERVO_SETTLE_MS);
  }

  // Trigger ultrasonic
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout

  // Restore previous angle
  if (savedAngle != checkAngle) {
    myServo.write(savedAngle);
    delay(SERVO_SETTLE_MS);
  }

  // No echo → assume clear
  if (duration == 0) {
    Serial.println("No echo – assuming clear");
    return true;
  }

  float distance = duration * 0.034 / 2.0;

  Serial.print("Distance ("); 
  Serial.print(checkAngle);
  Serial.print("°): ");
  Serial.print(distance);
  Serial.println(" cm");

  return (distance > OBSTACLE_DISTANCE_CM); // true = safe
}

